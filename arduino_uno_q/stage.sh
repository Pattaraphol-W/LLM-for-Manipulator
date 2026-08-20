#!/usr/bin/env bash
# Rebuild the deploy staging directory on the laptop: llama.cpp arm64 libs, the
# three GGUF models, the cross-compiled binary, and a test image. Everything is
# fetched here because the board's own network is ~1.3 KB/s.
#
#   ./stage.sh [dest]     # default: ./stage  (gitignored)
set -euo pipefail

DEST="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/stage}"
LLAMA_VERSION="${LLAMA_VERSION:-b10514}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$DEST/models" "$DEST/libslim"

if [ -z "$(ls -A "$DEST/libslim" 2>/dev/null)" ]; then
  echo "==> llama.cpp libs $LLAMA_VERSION (trixie/cpu/arm64)"
  tmp="$(mktemp -d)"
  curl -fL --retry 3 -o "$tmp/lib.tar.gz" \
    "https://github.com/hybridgroup/llama-cpp-builder/releases/download/$LLAMA_VERSION/llama-$LLAMA_VERSION-bin-ubuntu-trixie-cpu-arm64.tar.gz"
  tar -xzf "$tmp/lib.tar.gz" -C "$tmp"
  # Only the shared objects are needed; the llama-* CLI binaries are not.
  find "$tmp" -name '*.so*' -exec cp {} "$DEST/libslim/" \;
  rm -rf "$tmp"
fi

echo "==> models"
for u in \
  "https://huggingface.co/ggml-org/SmolVLM2-500M-Video-Instruct-GGUF/resolve/main/SmolVLM2-500M-Video-Instruct-Q8_0.gguf" \
  "https://huggingface.co/ggml-org/SmolVLM2-500M-Video-Instruct-GGUF/resolve/main/mmproj-SmolVLM2-500M-Video-Instruct-Q8_0.gguf" \
  "https://huggingface.co/bartowski/SmolLM2-135M-Instruct-GGUF/resolve/main/SmolLM2-135M-Instruct-Q4_K_M.gguf" ; do
  n="$(basename "$u")"
  [ -f "$DEST/models/$n" ] && { echo "    have $n"; continue; }
  echo "    $n"
  curl -fL --retry 3 -sS -o "$DEST/models/$n" "$u"
done

if [ ! -f "$DEST/bin/ffmpeg" ]; then
  echo "==> static arm64 ffmpeg (no apt/sudo needed on the board)"
  mkdir -p "$DEST/bin"
  tmp="$(mktemp -d)"
  curl -fL --retry 5 --retry-all-errors -C - \
    -o "$tmp/ffmpeg.tar.xz" "https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-arm64-static.tar.xz"
  tar -xJf "$tmp/ffmpeg.tar.xz" -C "$tmp" --strip-components=1
  cp "$tmp/ffmpeg" "$DEST/bin/ffmpeg" && chmod +x "$DEST/bin/ffmpeg"
  rm -rf "$tmp"
fi

echo "==> cross-compiling vlmcam for linux/arm64"
( cd "$HERE/vlmcam" && GOOS=linux GOARCH=arm64 go build -o "$DEST/vlmcam-arm64" . )

if [ ! -f "$DEST/test_512.jpg" ]; then
  echo "==> test image"
  curl -fsSL -o "$DEST/test_image.jpg" \
    "https://raw.githubusercontent.com/hybridgroup/yzma/v1.23.0/images/domestic_llama.jpg" || true
  python3 -c "
from PIL import Image; im=Image.open('$DEST/test_image.jpg')
im.resize((512, round(im.height*512/im.width)), Image.LANCZOS).save('$DEST/test_512.jpg', quality=88)" 2>/dev/null || \
    echo "    (no PIL; skipping the 512px resize)"
fi

du -sh "$DEST"
echo "==> stage ready: $DEST"
echo "    next: ./deploy_adb.sh --stage $DEST"
