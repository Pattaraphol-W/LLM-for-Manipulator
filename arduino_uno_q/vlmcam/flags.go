package main

import (
	"errors"
	"flag"
	"fmt"
	"os"
	"time"
)

var (
	modelFile    *string
	projFile     *string
	libPath      *string
	prompt       *string
	systemPrompt *string
	template     *string
	verbose      *bool

	device     *string
	imageFile  *string
	resolution *string
	framePath  *string
	skipFrames *int
	captureBin *string
	warmup     *string
	minLuma    *float64
	interval   *time.Duration
	once       *bool

	temperature *float64
	topK        *int
	topP        *float64
	minP        *float64
	contextSize *int
	batchSize   *int
	threads     *int
	gpuLayers   *int
	visionGPU   *bool
	imgMaxTok   *int
	predictSize *int
)

func showUsage() {
	fmt.Println(`
Usage:
  vlmcam -model <vlm.gguf> -mmproj <mmproj.gguf> [-p "prompt"] [-interval 5s]

The llama.cpp library directory comes from -lib, or from $YZMA_LIB if -lib is
not given. Run with -1 to describe a single frame and exit.

Flags:`)
	flag.PrintDefaults()
}

func handleFlags() error {
	modelFile = flag.String("model", "", "VLM model .gguf file (required)")
	projFile = flag.String("mmproj", "", "multimodal projector .gguf file (required)")
	libPath = flag.String("lib", "", "path to llama.cpp library files (default $YZMA_LIB)")
	prompt = flag.String("p", "What is in this picture?", "prompt applied to every frame")
	systemPrompt = flag.String("sys", "", "system prompt")
	template = flag.String("template", "", "chat template name (default: the model's own)")
	verbose = flag.Bool("v", false, "verbose llama.cpp logging")

	device = flag.String("device", "/dev/video0", "webcam device")
	imageFile = flag.String("image", "", "use this image file instead of capturing from a webcam")
	resolution = flag.String("res", "640x480", "capture resolution")
	framePath = flag.String("frame", "/tmp/vlmcam_frame.jpg", "path for the captured frame")
	skipFrames = flag.Int("skip", 8, "frames to discard while the sensor settles")
	captureBin = flag.String("capture-bin", "", "explicit fswebcam/ffmpeg binary to capture with")
	warmup = flag.String("warmup", "2", "seconds of frames to run through before keeping one (ffmpeg)")
	minLuma = flag.Float64("min-luma", 8.0, "reject frames darker than this mean luma (0-255) and retry")
	interval = flag.Duration("interval", 5*time.Second, "delay between frames")
	once = flag.Bool("1", false, "describe a single frame, then exit")

	temperature = flag.Float64("temp", 0.8, "temperature")
	topK = flag.Int("top-k", 40, "top-k")
	topP = flag.Float64("top-p", 0.9, "top-p")
	minP = flag.Float64("min-p", 0.1, "min-p")
	contextSize = flag.Int("c", 4096, "context size")
	batchSize = flag.Int("b", 2048, "max batch size")
	threads = flag.Int("threads", 0, "generation threads (0 = llama.cpp default)")
	gpuLayers = flag.Int("gpu-layers", 0, "LLM layers to offload to GPU (needs a Vulkan YZMA_LIB)")
	visionGPU = flag.Bool("vision-gpu", false, "run the vision encoder on the GPU too (crashes Adreno A702)")
	imgMaxTok = flag.Int("image-max-tokens", 0, "cap image tokens (0 = model default); fewer tokens = faster")
	predictSize = flag.Int("n", 128, "max tokens per description (<=0 for unlimited)")

	flag.Parse()

	if *modelFile == "" || *projFile == "" {
		return errors.New("both -model and -mmproj are required")
	}
	for _, f := range []string{*modelFile, *projFile} {
		if _, err := os.Stat(f); err != nil {
			return fmt.Errorf("cannot read %s: %w", f, err)
		}
	}
	return nil
}
