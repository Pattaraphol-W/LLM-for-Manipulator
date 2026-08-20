# Local Vision-Language Model on the Arduino UNO Q

Running a vision-language model (VLM) entirely on-device on an Arduino UNO Q, fed by a live
USB webcam. No cloud, no inference server — the model runs on the board's own CPU.

Built and measured 2026-08-20/21. Everything below was verified by running it on real
hardware; where something is assumed rather than measured, it says so.

---

## 1. What we planned

Take the Arduino Project Hub tutorial *"Running local LLMs and VLMs on the Arduino UNO Q with
yzma"* — which describes a VLM captioning a **bundled static JPEG** — and extend it to
**live webcam frames**, then evaluate whether it is fast enough to be useful for robotics
perception.

The tutorial's gap: it has no capture step at all. Everything from the camera onward is new.

## 2. What we used

| Layer | Choice | Notes |
|---|---|---|
| Board | Arduino UNO Q | Qualcomm QCM2290, 4× ARM64, 3.6 GB RAM, Adreno A702 |
| OS | Debian 13 "trixie" arm64 | Full Linux; user `arduino`, hostname `Batboard` |
| Inference engine | **llama.cpp `b10514`** | Prebuilt ARM64 shared libraries |
| Binding | **yzma v1.23.0** | Go wrapper; `purego`+`libffi` `dlopen`, **no cgo** |
| VLM | **SmolVLM2-500M-Video-Instruct**, Q8_0 GGUF | 417 MB |
| Projector | mmproj-SmolVLM2-500M, Q8_0 | 104 MB — the vision tower |
| Text LLM | SmolLM2-135M-Instruct, Q4_K_M | 101 MB, present but unused |
| Capture | static **ffmpeg 7.0.2** arm64, v4l2 | Sideloaded; no apt, no sudo |
| Application | `vlmcam` — custom Go program | ~350 lines, uses llama.cpp's `mtmd` API |

Because yzma is pure Go and loads llama.cpp at runtime, `vlmcam` **cross-compiles on a laptop**
(`GOOS=linux GOARCH=arm64`) into a single 3.5 MB binary. **No toolchain is needed on the board.**

## 3. How it works

```
webcam ──ffmpeg/v4l2 (2s warmup)──> JPEG ──mean-luma check──> reject/retry if black
                                              │
       chat template + image marker ──> mtmd.Tokenize   (text tokens + image chunk)
                                              │
                            mtmd.HelperEvalChunks   ← vision encoder + prefill (~47 s)
                                              │
                     sampling loop: SamplerSample → VocabIsEOG? → TokenToPiece → Decode
                                              │                        (~0.12 s/token)
                     between frames: MemoryClear (KV cache) + SamplerReset
```

The model and projector are loaded **once** and stay resident; each frame clears the KV cache
so descriptions are independent rather than conditioned on previous frames.

## 4. Running it

```bash
# on the board
~/arduino_uno_q/run_vlm.sh -1              # one frame from the webcam
~/arduino_uno_q/run_vlm.sh -interval 5s    # continuous loop
~/arduino_uno_q/run_vlm.sh -image foo.jpg -1   # static image, no camera

# with the live web UI, then browse to http://<board-ip>:8080
~/arduino_uno_q/run_vlm.sh -serve :8080 -interval 5s
```

### Seeing what the model sees

`-serve` starts a small HTTP server (Go stdlib, no dependencies) inside the same process. The
page shows **the exact JPEG handed to the model**, the description streaming in token by
token, and the live mean-luma reading with a warning when a frame is too dark. Endpoints are
`/` (page), `/frame.jpg` (raw frame) and `/api` (JSON state); the browser polls `/api` and
reloads the image when the frame counter changes.

This exists because the board is headless — `card0-DP-1` is disconnected, and installing an
image viewer would need both sudo and a working network. A browser on any device on the same
network needs neither.

Useful when adjusting lighting: the luma figure updates live, which is far quicker than
capture → copy → look → repeat.

**There is no authentication.** Anyone on the same network can view the feed. Fine on a desk,
not on a shared or public network.

From a laptop: `./stage.sh` fetches libraries + models and cross-compiles; `./deploy_adb.sh`
pushes over USB, or plain `scp` over the network.

## 5. Results

Frame cost decomposes into a large fixed cost and a small per-token cost:

| Stage | Cost |
|---|---|
| Model load | 0.9 s (once) |
| **Image encode + prefill** | **~47 s** |
| Token generation | 0.12 s/token |

Measured by holding the prompt fixed and varying output length: 60 tokens took 71.7 s, 25
tokens took 67.6 s — a 35-token difference costing only 4.1 s. **Almost all the time is the
vision encoder, not text generation.**

### Input resolution sweep (same scene, 25 tokens)

| Input | Time |
|---|---|
| 960×720 | 67.6 s |
| **512×384** | **48.0 s** |
| 384×288 | 47.4 s |
| 256×192 | 46.6 s |

Above 512 px costs real time; below it buys nothing, because the model pads to a fixed
internal resolution. **512 px is the sweet spot.**

### Backend comparison (512 px, 25 tokens)

| Configuration | Result |
|---|---|
| CPU (4 threads) | **56.0 s** |
| Vulkan / Adreno A702, full offload | **GPU lockup — crash** |
| Vulkan, LLM layers only (vision on CPU) | **GPU lockup — crash** |
| CPU + `-image-max-tokens 64` | 48.5 s (≈13 % faster) |

Run-to-run variance is roughly ±15 % (the same 512 px config measured 48.0 s and 56.0 s on
different occasions), so treat small differences as noise.

Setting `-threads 4` explicitly changed nothing — llama.cpp already uses all four cores.

## 6. What worked

- **The whole pipeline runs on-device.** Camera → VLM → text description, ~50 s/frame, no network.
- **Cross-compilation.** Pure-Go binding meant zero build tooling on the board.
- **Sideloading everything.** No `apt`, no `sudo`, no package installs; all under `$HOME`.
- **Resident model + KV-cache clearing.** Verified across consecutive frames: independent
  descriptions, no bleed between them.
- **Frame validation.** Measuring brightness before inference turned a silent failure into a
  loud one (see below).

## 7. What failed

**The tutorial's pinned versions do not work.** yzma v1.9.0 binds the flat C symbol
`llama_params_fit`, which current llama.cpp no longer exports (it moved, C++-mangled, into
`libllama-fit-params-impl.so`). Loading dies instantly with `undefined symbol`. Fixed by
upgrading to v1.23.0, which changed the API (`mtmd.BitmapInitFromFile` gained a third
argument and now returns a wrapper struct).

**`yzma install` is broken independently of that.** It queries the llama.cpp GitHub API for
the *latest* release, then looks for a matching ARM64 build in a second repository that lags
behind → 404. Pin with `--version`, or sideload.

**GPU acceleration is a dead end on this board.** The Adreno A702 is detected correctly by
Mesa's Turnip driver, but any offload — even language-model layers only — hangs the GPU:

```
hangcheck detected gpu lockup rb 0
gpu fault: dir=READ type=TRANSLATION source=UCHE
terminate: vk::DeviceLostError — vk::Queue::submit: ErrorDeviceLost
```

Consistent with what the driver reports: `matrix cores: none`, `int dot: 0`. This entry-level
GPU has no matrix or integer-dot acceleration, which is exactly what quantized inference
needs. **CPU-only is the correct configuration here.**

**The black-frame trap — the most important finding.** A USB webcam's auto-exposure ramps over
*wall-clock time*, not a fixed number of frames. Capture immediately and you get a black JPEG:

| Capture | Mean luma (YAVG) |
|---|---|
| First frame, cold camera | **0.02** (pure black) |
| 2-second warmup | 88 – 150 |

A VLM handed a black image **does not report darkness — it fabricates a detailed scene**:

> *"a black and white photograph … a person standing in front of a large, open book"*
> *"a young man … in a white and pink sports jersey … holding a basketball"*

Both from the same blank input, in the same room, with no error or warning of any kind. This
is the single biggest practical risk in a VLM perception pipeline: **silent garbage in,
confident garbage out.** `vlmcam` now decodes every frame, computes mean luma, retries up to
three times, and fails loudly rather than describing a void.

Note that **file size is a useless test** — a dark frame is full of sensor noise and
compresses to about the same size as a real one (11,805 vs 10,067 bytes). Brightness is the test.

**Minor traps:** `/dev/video0` was initially the Qualcomm Venus *hardware codec*, not a camera;
plugging in a USB webcam renumbered the codec to `video2/3` and took `video0` itself. The
camera also advertises no 512×384 mode, so the wrapper requests 640×480.

## 7b. Output quality

Speed is only half the story. A 20-run repeatability test on a single unchanging scene found
pairwise agreement between runs of **0.12** — successive descriptions of one identical image
share about an eighth of their content words — while latency stayed stable to ±1.3 s.

The model reliably reports texture and colour (shaggy object 70%, something red 55%, dark
scene 60%) but attaches a different noun to the red object nearly every run. Its one
consistent failure is **alive vs. toy**: 18/20 runs describe a live animal when the subject is
a plush toy — exactly the error that matters for a robot.

Note that the frame was 64% near-black, so the scene shares the blame; and an initial version
of that analysis scored the model against the operator's *recollection* rather than the
captured frame, which inflated the hallucination rate roughly twofold.

Full data, evaluator and analysis: [`experiments/2026-08-20-static-scene`](../experiments/2026-08-20-static-scene/).

## 8. Conclusions

**It works, and it is far too slow to be a live perception system.** At ~50 s/frame — with
~47 s of that in the vision encoder — no amount of prompt tuning or resolution reduction
changes the picture, and the GPU cannot help.

The right conclusion is not "make it faster" but **use it differently**: a VLM on this class
of hardware is not a sensor, it is an *oracle you consult occasionally*. For a manipulator:

1. **Gate it.** Frame-difference detection costs milliseconds. Only wake the VLM when the
   scene actually changes; most frames are redundant.
2. **Two-tier perception.** Cheap classical CV (OpenCV, ArUco markers, blob/motion tracking)
   at 30 fps for control; the VLM occasionally, for semantics — *"what objects are on the
   table?"* — then track those cheaply.
3. **A smaller model** (SmolVLM-256M) is ~2× faster with worse descriptions; diminishing
   returns compared to 1 and 2.

Fifty seconds is entirely acceptable for a question asked once per task. It is useless at
30 fps. Design around that.

## 9. Files

| File | Runs on | Purpose |
|---|---|---|
| `vlmcam/` | laptop → board | The Go program (source; cross-compiled to arm64). `server.go` is the optional live web UI. |
| `run_vlm.sh` | board | Wrapper: sets `YZMA_LIB`, model paths, 640×480 capture |
| `stage.sh` | laptop | Downloads libs + models + static ffmpeg, cross-compiles |
| `deploy_adb.sh` | laptop | Pushes to the board over USB (ADB); `--check` reports state |
| `push.sh` | laptop | SSH/scp deploy alternative |
| `provision_unoq.sh` | board | Online install path — needs a board with working internet |

### Accessing the board

| Method | When |
|---|---|
| **ADB over USB** | Board tethered to a PC. App Lab ships the `adb` binary. |
| **SSH** | Board on a network. Works well; used for most of this work. |
| **App Lab** | Arduino/MCU-side development — **not needed for any of this**. |

The UNO Q has **one USB-C port**, which is either a device (talking to a PC) or a host
(running a webcam), never both. With a webcam attached the board must be reached over the
network; a powered USB-C dock provides both plus Ethernet.
