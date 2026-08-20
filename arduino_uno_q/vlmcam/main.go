// vlmcam runs SmolVLM2 (or any llama.cpp-compatible VLM) against live webcam
// frames on the Arduino UNO Q's Linux side.
//
// The stock yzma example loads the model, describes one image, and exits. That
// costs a full model load (hundreds of MB off eMMC) for every frame, which
// dominates the runtime on an ARM CPU. This keeps the model and the multimodal
// projector resident and loops capture -> infer -> print, clearing the KV cache
// between frames so each description is independent.
//
// This is frame-on-demand inference, not video: expect multiple seconds per
// frame for a 500M-parameter model on this hardware.
package main

import (
	"fmt"
	"image/jpeg"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"
	"time"

	"github.com/hybridgroup/yzma/pkg/llama"
	"github.com/hybridgroup/yzma/pkg/mtmd"
)

var (
	model   llama.Model
	lctx    llama.Context
	vocab   llama.Vocab
	sampler llama.Sampler
	mtmdCtx mtmd.Context
	memory  llama.Memory

	messages []llama.ChatMessage
)

func main() {
	if err := handleFlags(); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		showUsage()
		os.Exit(2)
	}

	lib := *libPath
	if lib == "" {
		lib = os.Getenv("YZMA_LIB")
	}
	if lib == "" {
		fmt.Fprintln(os.Stderr, "error: no library path. Pass -lib or set YZMA_LIB (see provision_unoq.sh).")
		os.Exit(1)
	}

	if err := llama.Load(lib); err != nil {
		fmt.Fprintln(os.Stderr, "unable to load llama library:", err)
		os.Exit(1)
	}
	if err := mtmd.Load(lib); err != nil {
		fmt.Fprintln(os.Stderr, "unable to load mtmd library:", err)
		os.Exit(1)
	}

	if !*verbose {
		llama.LogSet(llama.LogSilent())
		mtmd.LogSet(llama.LogSilent())
	}

	llama.Init()
	defer llama.Close()

	fmt.Println("loading model:", *modelFile)
	loadStart := time.Now()

	var err error
	mp := llama.ModelDefaultParams()
	mp.NGpuLayers = int32(*gpuLayers)
	model, err = llama.ModelLoadFromFile(*modelFile, mp)
	if err != nil || model == 0 {
		fmt.Fprintln(os.Stderr, "unable to load model from file:", *modelFile, err)
		os.Exit(1)
	}
	defer llama.ModelFree(model)

	ctxParams := llama.ContextDefaultParams()
	ctxParams.NCtx = uint32(*contextSize)
	ctxParams.NBatch = uint32(*batchSize)
	if *threads > 0 {
		ctxParams.NThreads = int32(*threads)
		ctxParams.NThreadsBatch = int32(*threads)
	}

	lctx, err = llama.InitFromModel(model, ctxParams)
	if err != nil {
		fmt.Fprintln(os.Stderr, "unable to initialize context from model:", err)
		os.Exit(1)
	}
	defer llama.Free(lctx)

	vocab = llama.ModelGetVocab(model)

	sp := llama.DefaultSamplerParams()
	sp.Temp = float32(*temperature)
	sp.TopK = int32(*topK)
	sp.TopP = float32(*topP)
	sp.MinP = float32(*minP)
	sampler = llama.NewSampler(model, llama.DefaultSamplers, sp)
	defer llama.SamplerFree(sampler)

	// The vision encoder is kept on the CPU by default: on this board's Adreno
	// A702 the Turnip driver dies inside clip with vk::DeviceLostError, taking
	// the whole process with it. The language model can still use the GPU.
	mctxParams := mtmd.ContextParamsDefault()
	mctxParams.UseGPU = *visionGPU
	if *imgMaxTok > 0 {
		mctxParams.ImageMaxTokens = int32(*imgMaxTok)
	}
	mtmdCtx, err = mtmd.InitFromFile(*projFile, model, mctxParams)
	if err != nil {
		fmt.Fprintln(os.Stderr, "unable to initialize projector from file:", err)
		os.Exit(1)
	}
	defer mtmd.Free(mtmdCtx)

	if !mtmd.SupportVision(mtmdCtx) {
		fmt.Fprintln(os.Stderr, "error: this projector reports no vision support")
		os.Exit(1)
	}

	// Reused between frames to drop the previous frame's tokens from the cache.
	memory, err = llama.GetMemory(lctx)
	if err != nil {
		fmt.Fprintln(os.Stderr, "unable to get context memory:", err)
		os.Exit(1)
	}

	if *template == "" {
		*template = llama.ModelChatTemplate(model, "")
	}

	fmt.Printf("model ready in %s\n", time.Since(loadStart).Round(time.Millisecond))

	// Ctrl-C should unwind through the defers above so the native allocations
	// are released, rather than dropping out of the loop mid-decode.
	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)
	done := false
	go func() {
		<-stop
		fmt.Println("\nstopping after the current frame...")
		done = true
	}()

	for !done {
		frame := *framePath
		if *imageFile != "" {
			// Static-image mode: no camera involved. Useful for validating the
			// model path on a board with no webcam attached.
			frame = *imageFile
		} else if err := captureFrame(); err != nil {
			fmt.Fprintln(os.Stderr, "capture failed:", err)
			if *once {
				os.Exit(1)
			}
			time.Sleep(*interval)
			continue
		}

		fmt.Printf("\n[%s] ", time.Now().Format("15:04:05"))
		start := time.Now()
		if err := describe(frame); err != nil {
			fmt.Fprintln(os.Stderr, "inference failed:", err)
		}
		fmt.Printf("\n  (%s)\n", time.Since(start).Round(time.Millisecond))

		if *once {
			break
		}
		time.Sleep(*interval)
	}
}

// captureFrame grabs a single still from the webcam. fswebcam is preferred
// because -S skips frames while the sensor settles on exposure; a cold camera's
// first frame is usually black or badly exposed, which the VLM will faithfully
// describe as darkness.
func captureFrame() error {
	// Retry, because a cold sensor can hand back a pure-black frame even after a
	// warmup period -- and a VLM given a black image does not report darkness, it
	// invents a detailed scene. Measuring the frame is the only reliable defence.
	var last float64
	for attempt := 1; attempt <= 3; attempt++ {
		if err := grab(); err != nil {
			return err
		}
		luma, err := meanLuma(*framePath)
		if err != nil {
			return fmt.Errorf("reading captured frame: %w", err)
		}
		last = luma
		if luma >= *minLuma {
			if luma < 25 {
				fmt.Printf("(dim frame, mean luma %.1f) ", luma)
			}
			return nil
		}
		fmt.Fprintf(os.Stderr, "frame %d was black (mean luma %.2f), retrying...\n", attempt, luma)
	}
	return fmt.Errorf("camera returned a black frame 3 times (last mean luma %.2f); "+
		"check lighting, lens cap, and that %s is the right device", last, *device)
}

// grab runs one capture into framePath.
func grab() error {
	tool, kind, err := captureTool()
	if err != nil {
		return err
	}

	var cmd *exec.Cmd
	if kind == "fswebcam" {
		cmd = exec.Command(tool,
			"-d", *device, "-r", *resolution, "-S", fmt.Sprint(*skipFrames),
			"--no-banner", "--quiet", *framePath)
	} else {
		// Run the camera for a fixed wall-clock warmup and let every frame
		// overwrite the last (-update 1), so what survives is the final settled
		// frame. A frame *count* is not enough: the sensor's auto-exposure ramp
		// is measured in time, not frames.
		cmd = exec.Command(tool, "-y", "-loglevel", "error",
			"-f", "v4l2", "-video_size", *resolution, "-i", *device,
			"-t", *warmup, "-update", "1", *framePath)
	}
	out, err := cmd.CombinedOutput()
	if err != nil {
		return fmt.Errorf("%s: %v: %s", tool, err, strings.TrimSpace(string(out)))
	}
	return nil
}

// meanLuma returns the average brightness (0-255) of a JPEG, sampling every
// 4th pixel. Used to tell a real frame from a black one.
func meanLuma(path string) (float64, error) {
	f, err := os.Open(path)
	if err != nil {
		return 0, err
	}
	defer f.Close()

	img, err := jpeg.Decode(f)
	if err != nil {
		return 0, err
	}

	b := img.Bounds()
	var sum float64
	var n int
	for y := b.Min.Y; y < b.Max.Y; y += 4 {
		for x := b.Min.X; x < b.Max.X; x += 4 {
			r, g, bl, _ := img.At(x, y).RGBA()
			sum += 0.299*float64(r>>8) + 0.587*float64(g>>8) + 0.114*float64(bl>>8)
			n++
		}
	}
	if n == 0 {
		return 0, fmt.Errorf("empty image")
	}
	return sum / float64(n), nil
}

// captureTool picks the capture binary. A copy shipped next to this executable
// (bin/ffmpeg) wins, because on the UNO Q neither fswebcam nor ffmpeg is
// installed and apt needs both sudo and a working network -- so a static build
// is dropped in alongside the binary instead.
func captureTool() (path, kind string, err error) {
	if *captureBin != "" {
		return *captureBin, filepath.Base(*captureBin), nil
	}
	if exe, e := os.Executable(); e == nil {
		local := filepath.Join(filepath.Dir(exe), "bin", "ffmpeg")
		if st, e := os.Stat(local); e == nil && !st.IsDir() {
			return local, "ffmpeg", nil
		}
	}
	if p, e := exec.LookPath("fswebcam"); e == nil {
		return p, "fswebcam", nil
	}
	if p, e := exec.LookPath("ffmpeg"); e == nil {
		return p, "ffmpeg", nil
	}
	return "", "", fmt.Errorf("no capture tool: put a static ffmpeg at bin/ffmpeg next to vlmcam, or pass -capture-bin")
}

// describe runs one image + prompt through the model and streams the answer to
// stdout. Every frame starts from a clean cache and sampler so descriptions do
// not bleed into each other.
func describe(image string) error {
	if err := llama.MemoryClear(memory, true); err != nil {
		return fmt.Errorf("clearing memory: %w", err)
	}
	llama.SamplerReset(sampler)

	messages = messages[:0]
	if *systemPrompt != "" {
		messages = append(messages, llama.NewChatMessage("system", *systemPrompt))
	}
	messages = append(messages, llama.NewChatMessage("user", mtmd.DefaultMarker()+*prompt))

	// v1.23 returns a wrapper (bitmap + optional video context) and takes a
	// "placeholder" flag; we only ever feed it single still frames.
	bw := mtmd.BitmapInitFromFile(mtmdCtx, image, false)
	if bw.Bitmap == 0 {
		return fmt.Errorf("could not read image %s", image)
	}
	defer mtmd.BitmapFree(bw.Bitmap)

	chunks := mtmd.InputChunksInit()
	defer mtmd.InputChunksFree(chunks)

	input := mtmd.NewInputText(chatTemplate(true), true, true)
	if res := mtmd.Tokenize(mtmdCtx, chunks, input, []mtmd.Bitmap{bw.Bitmap}); res != 0 {
		return fmt.Errorf("tokenize returned %d", res)
	}

	var n llama.Pos
	if res := mtmd.HelperEvalChunks(mtmdCtx, lctx, chunks, 0, 0, int32(*batchSize), true, &n); res != 0 {
		return fmt.Errorf("eval chunks returned %d", res)
	}

	limit := *predictSize
	if limit <= 0 {
		limit = llama.MaxToken
	}
	buf := make([]byte, 128)
	for i := 0; i < limit; i++ {
		token := llama.SamplerSample(sampler, lctx, -1)
		if llama.VocabIsEOG(vocab, token) {
			break
		}

		l := llama.TokenToPiece(vocab, token, buf, 0, true)
		fmt.Print(string(buf[:l]))

		batch := llama.BatchGetOne([]llama.Token{token})
		batch.Pos = &n
		if res, err := llama.Decode(lctx, batch); err != nil {
			return fmt.Errorf("decode: %w", err)
		} else if res != 0 {
			return fmt.Errorf("decode returned %d (context full at %d tokens?)", res, n)
		}
		n++
	}
	return nil
}

func chatTemplate(add bool) string {
	buf := make([]byte, 4096)
	l := llama.ChatApplyTemplate(*template, messages, add, buf)
	return string(buf[:l])
}
