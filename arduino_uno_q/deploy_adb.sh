#!/usr/bin/env bash
# Deploy the local VLM stack to an Arduino UNO Q over ADB (USB).
#
# Why ADB and not ssh/scp: the UNO Q exposes an ADB interface over USB-C (it is a
# Qualcomm Linux target). On this setup that is also the only *reliable* link --
# the board's own Wi-Fi measured ~1.3 KB/s, which makes downloading models or Go
# modules on the board itself impractical. So everything is fetched on the
# laptop's connection and pushed over USB, and nothing here needs sudo on the
# board: it all lands under $HOME.
#
#   ./deploy_adb.sh --stage <dir>   # dir containing models/ and libslim/ and vlmcam-arm64
#   ./deploy_adb.sh --check         # just report what is already on the board
set -euo pipefail

REMOTE_HOME="/home/arduino"
REMOTE_DIR="$REMOTE_HOME/arduino_uno_q"
STAGE=""
CHECK_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --stage) STAGE="$2"; shift 2 ;;
    --check) CHECK_ONLY=1; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# Prefer a native adb; fall back to the one Arduino App Lab ships on Windows,
# reached through WSL interop.
find_adb() {
  if command -v adb >/dev/null; then command -v adb; return; fi
  local applab="/mnt/c/Users/$(cmd.exe /c 'echo %USERNAME%' 2>/dev/null | tr -d '\r\n')/AppData/Local/Arduino15/packages/arduino/tools/adb"
  local candidate
  candidate="$(ls -d "$applab"/*/adb.exe 2>/dev/null | sort -V | tail -1 || true)"
  [ -n "$candidate" ] && { echo "$candidate"; return; }
  return 1
}

ADB="$(find_adb)" || { echo "no adb found. Install android-tools-adb, or Arduino App Lab on Windows." >&2; exit 1; }
echo "==> adb: $ADB"

# tr -d '\r': the App Lab adb is a Windows binary and emits CRLF through interop,
# which otherwise makes the state field compare as "device\r" and never match.
DEV="$("$ADB" devices | tr -d '\r' | awk 'NR>1 && $2=="device" {print $1; exit}')"
[ -n "$DEV" ] || { echo "no authorized device. Check the USB cable and that App Lab sees the board." >&2; exit 1; }
echo "==> device: $DEV"

sh_() { "$ADB" shell "$@" 2>&1 | tr -d '\r'; }

if [ "$CHECK_ONLY" -eq 1 ]; then
  sh_ "echo '--- board ---'; uname -m; . /etc/os-release && echo \$PRETTY_NAME; free -h | head -2; df -h / | tail -1
       echo '--- installed ---'; ls -la $REMOTE_DIR 2>/dev/null | head
       echo '--- models ---'; ls -lh $REMOTE_HOME/models 2>/dev/null || echo none
       echo '--- libs ---'; ls $REMOTE_HOME/yzma/lib 2>/dev/null | wc -l
       echo '--- cameras ---'; for d in /dev/video*; do echo \"\$d: \$(v4l2-ctl -d \$d --info 2>/dev/null | grep -m1 'Card type' || echo '?')\"; done"
  exit 0
fi

[ -n "$STAGE" ] || { echo "--stage <dir> is required (see README)" >&2; exit 2; }
[ -d "$STAGE" ] || { echo "stage dir not found: $STAGE" >&2; exit 1; }

echo "==> free space on board before:"
sh_ "df -h / | tail -1"

sh_ "mkdir -p $REMOTE_DIR $REMOTE_HOME/models $REMOTE_HOME/yzma/lib"

if [ -f "$STAGE/vlmcam-arm64" ]; then
  echo "==> pushing vlmcam binary"
  "$ADB" push "$STAGE/vlmcam-arm64" "$REMOTE_DIR/vlmcam" >/dev/null
  sh_ "chmod +x $REMOTE_DIR/vlmcam"
fi

if [ -f "$STAGE/bin/ffmpeg" ]; then
  echo "==> pushing static ffmpeg"
  sh_ "mkdir -p $REMOTE_DIR/bin"
  "$ADB" push "$STAGE/bin/ffmpeg" "$REMOTE_DIR/bin/ffmpeg" >/dev/null
  sh_ "chmod +x $REMOTE_DIR/bin/ffmpeg"
fi

if [ -d "$STAGE/libslim" ]; then
  echo "==> pushing llama.cpp shared libraries"
  "$ADB" push "$STAGE/libslim/." "$REMOTE_HOME/yzma/lib/" >/dev/null
fi

if [ -d "$STAGE/models" ]; then
  echo "==> pushing models (this is the slow part)"
  for m in "$STAGE"/models/*.gguf; do
    [ -e "$m" ] || continue
    name="$(basename "$m")"
    if sh_ "test -f $REMOTE_HOME/models/$name && echo yes" | grep -q yes; then
      echo "    already on board: $name"
    else
      echo "    $name ($(du -h "$m" | cut -f1))"
      "$ADB" push "$m" "$REMOTE_HOME/models/$name" >/dev/null
    fi
  done
fi

for s in run_vlm.sh; do
  [ -f "$(dirname "$0")/$s" ] && "$ADB" push "$(dirname "$0")/$s" "$REMOTE_DIR/$s" >/dev/null && sh_ "chmod +x $REMOTE_DIR/$s"
done

echo "==> persisting YZMA_LIB on the board"
sh_ "grep -q 'YZMA_LIB' $REMOTE_HOME/.bashrc || printf '\nexport YZMA_LIB=%s/yzma/lib\n' '$REMOTE_HOME' >> $REMOTE_HOME/.bashrc"

echo "==> free space on board after:"
sh_ "df -h / | tail -1"

cat <<NEXT

Deployed. To run a single inference on the board:

  $ADB shell
  export YZMA_LIB=$REMOTE_HOME/yzma/lib
  cd $REMOTE_DIR
  ./vlmcam -model $REMOTE_HOME/models/SmolVLM2-500M-Video-Instruct-Q8_0.gguf \\
           -mmproj $REMOTE_HOME/models/mmproj-SmolVLM2-500M-Video-Instruct-Q8_0.gguf \\
           -image some.jpg -1

NEXT
