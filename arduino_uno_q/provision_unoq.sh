#!/usr/bin/env bash
# Provision an Arduino UNO Q (Debian side) to run local LLM/VLM inference with yzma.
# Run this ON THE BOARD, as the normal user (usually `arduino`), not as root.
#
#   ./provision_unoq.sh              # full setup: go, yzma, llama.cpp libs, models, webcam tools
#   ./provision_unoq.sh --no-models  # skip the ~800 MB model download
#
# Idempotent: re-running skips whatever is already in place.
set -euo pipefail

YZMA_VERSION="${YZMA_VERSION:-v1.23.0}"  # v1.9.0 (tutorial) is INCOMPATIBLE with current llama.cpp builds
LLAMA_VERSION="${LLAMA_VERSION:-b10514}" # pin: yzma asks for llama.cpp "latest", which 404s on the arm64 builder repo
YZMA_DIR="$HOME/yzma"
YZMA_LIB_DIR="$YZMA_DIR/lib"
MODELS_DIR="$HOME/models"
WANT_MODELS=1

for arg in "$@"; do
  case "$arg" in
    --no-models) WANT_MODELS=0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

[ "$(id -u)" -eq 0 ] && { echo "Run as your normal user, not root." >&2; exit 1; }

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

# --- 0. sanity: are we actually on an arm64 Debian board? ------------------
say "host check"
uname -m
. /etc/os-release && echo "$PRETTY_NAME"
if [ "$(uname -m)" != "aarch64" ]; then
  echo "!! This is not arm64. This script is meant to run ON the UNO Q, not on your laptop." >&2
  exit 1
fi

# --- 1. apt packages -------------------------------------------------------
say "installing apt packages (go, git, webcam tools)"
sudo apt-get update
sudo apt-get install -y golang git ca-certificates fswebcam ffmpeg v4l-utils

go version   # tutorial expects go1.24.4 linux/arm64 or newer

# --- 2. yzma CLI -----------------------------------------------------------
export PATH="$PATH:$(go env GOPATH)/bin"
if ! command -v yzma >/dev/null; then
  say "installing yzma CLI $YZMA_VERSION"
  go install "github.com/hybridgroup/yzma/cmd/yzma@$YZMA_VERSION"
else
  say "yzma already installed: $(yzma version 2>&1 | head -1)"
fi

# --- 3. yzma source checkout (for the bundled examples) --------------------
if [ ! -d "$YZMA_DIR/.git" ]; then
  say "cloning yzma source to $YZMA_DIR"
  git clone https://github.com/hybridgroup/yzma "$YZMA_DIR"
  git -C "$YZMA_DIR" checkout "$YZMA_VERSION"
fi
mkdir -p "$YZMA_LIB_DIR"

# --- 4. persist env vars ---------------------------------------------------
MARKER="# >>> yzma env (managed by provision_unoq.sh) >>>"
if ! grep -qF "$MARKER" "$HOME/.bashrc"; then
  say "adding PATH and YZMA_LIB to ~/.bashrc"
  cat >> "$HOME/.bashrc" <<BASHRC

$MARKER
export PATH=\$PATH:\$(go env GOPATH)/bin
export YZMA_LIB=$YZMA_LIB_DIR
# <<< yzma env <<<
BASHRC
fi
export YZMA_LIB="$YZMA_LIB_DIR"

# --- 5. llama.cpp shared libraries ----------------------------------------
if [ -z "$(ls -A "$YZMA_LIB_DIR" 2>/dev/null)" ]; then
  say "installing llama.cpp libs (cpu / trixie)"
  yzma install -u --processor cpu --os trixie --version "$LLAMA_VERSION" --lib "$YZMA_LIB_DIR"
else
  say "llama.cpp libs already present in $YZMA_LIB_DIR"
fi
ls -lh "$YZMA_LIB_DIR" | head

# --- 6. models -------------------------------------------------------------
if [ "$WANT_MODELS" -eq 1 ]; then
  mkdir -p "$MODELS_DIR"
  get_model() {
    local url="$1" name
    name="$(basename "$url")"
    if [ -f "$MODELS_DIR/$name" ]; then
      echo "  already have $name"
    else
      say "downloading $name"
      yzma model get -y -u "$url" -o "$MODELS_DIR"
    fi
  }
  get_model https://huggingface.co/bartowski/SmolLM2-135M-Instruct-GGUF/resolve/main/SmolLM2-135M-Instruct-Q4_K_M.gguf
  get_model https://huggingface.co/ggml-org/SmolVLM2-500M-Video-Instruct-GGUF/resolve/main/SmolVLM2-500M-Video-Instruct-Q8_0.gguf
  get_model https://huggingface.co/ggml-org/SmolVLM2-500M-Video-Instruct-GGUF/resolve/main/mmproj-SmolVLM2-500M-Video-Instruct-Q8_0.gguf
  du -sh "$MODELS_DIR"
fi

# --- 7. webcam check -------------------------------------------------------
say "webcam check"
if ls /dev/video* >/dev/null 2>&1; then
  ls -l /dev/video*
  v4l2-ctl --list-devices || true
  echo "Supported formats on /dev/video0:"
  v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null | head -20 || true
else
  echo "!! No /dev/video* device found."
  echo "   Plug in a USB webcam and re-run, or the webcam half of the pipeline will not work."
fi
if ! id -nG "$USER" | grep -qw video; then
  say "adding $USER to the video group (log out/in to take effect)"
  sudo usermod -aG video "$USER"
fi

say "done"
cat <<'NEXT'
Next steps (on the board):

  source ~/.bashrc

  # 1. baseline: static image, straight from the tutorial
  cd ~/yzma
  go run ./examples/vlm/ \
    -model ~/models/SmolVLM2-500M-Video-Instruct-Q8_0.gguf \
    -mmproj ~/models/mmproj-SmolVLM2-500M-Video-Instruct-Q8_0.gguf \
    -image ./images/domestic_llama.jpg -p "What is in this picture?" -v

  # 2. one webcam frame through the same path
  ~/arduino_uno_q/vlm_webcam.sh -1

  # 3. continuous loop, model stays resident in RAM (much faster per frame)
  cd ~/arduino_uno_q/vlmcam && go build . && ./vlmcam \
    -model ~/models/SmolVLM2-500M-Video-Instruct-Q8_0.gguf \
    -mmproj ~/models/mmproj-SmolVLM2-500M-Video-Instruct-Q8_0.gguf \
    -p "Describe what you see in one sentence." -interval 5s
NEXT
