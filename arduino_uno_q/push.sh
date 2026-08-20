#!/usr/bin/env bash
# Copy this directory to the UNO Q and (optionally) run the provisioner.
# Runs on the WSL/laptop side.
#
#   ./push.sh unoq                 # host from ~/.ssh/config, or user@ip
#   ./push.sh arduino@10.0.0.42
#   ./push.sh unoq --provision     # copy, then run provision_unoq.sh over ssh
set -euo pipefail

HOST="${1:-}"
[ -n "$HOST" ] || { echo "usage: $0 <ssh-host|user@ip> [--provision]" >&2; exit 2; }
PROVISION=0
[ "${2:-}" = "--provision" ] && PROVISION=1

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="arduino_uno_q"

echo "==> checking ssh to $HOST"
ssh -o ConnectTimeout=10 -o BatchMode=yes "$HOST" 'echo "connected: $(uname -m) $(. /etc/os-release && echo "$PRETTY_NAME")"'

echo "==> copying $SRC -> $HOST:~/$DEST"
ssh "$HOST" "mkdir -p ~/$DEST"
if command -v rsync >/dev/null && ssh "$HOST" 'command -v rsync >/dev/null'; then
  rsync -av --delete --exclude 'vlmcam/vlmcam' --exclude '*.jpg' "$SRC/" "$HOST:~/$DEST/"
else
  scp -r "$SRC/provision_unoq.sh" "$SRC/vlm_webcam.sh" "$SRC/README.md" "$SRC/vlmcam" "$HOST:~/$DEST/"
fi
ssh "$HOST" "chmod +x ~/$DEST/*.sh"

if [ "$PROVISION" -eq 1 ]; then
  echo "==> running provisioner on $HOST (this will ask for the board's sudo password)"
  ssh -t "$HOST" "~/$DEST/provision_unoq.sh"
fi

echo "==> done. ssh $HOST"
