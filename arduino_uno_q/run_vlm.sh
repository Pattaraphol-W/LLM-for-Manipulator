#!/usr/bin/env bash
# Convenience wrapper for vlmcam. Runs ON THE BOARD.
#   ./run_vlm.sh                          # loop, capturing from the webcam
#   ./run_vlm.sh -image test_512.jpg -1   # single static image
# Any extra flags are passed straight through to vlmcam.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export YZMA_LIB="${YZMA_LIB:-$HOME/yzma/lib}"
exec "$HERE/vlmcam" \
  -model "$HOME/models/SmolVLM2-500M-Video-Instruct-Q8_0.gguf" \
  -mmproj "$HOME/models/mmproj-SmolVLM2-500M-Video-Instruct-Q8_0.gguf" \
  -res 640x480 \
  "$@"
