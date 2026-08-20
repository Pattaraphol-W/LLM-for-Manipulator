#!/usr/bin/env bash
# Set up the build/dev environment for this workspace on Ubuntu 22.04 + ROS 2 Humble.
# Idempotent: safe to re-run. Needs sudo (it installs apt packages).
#
#   ./scripts/setup_env.sh              # base toolchain
#   ./scripts/setup_env.sh --moveit     # also install MoveIt 2 (~1 GB)
#
set -euo pipefail

WITH_MOVEIT=0
for arg in "$@"; do
  case "$arg" in
    --moveit) WITH_MOVEIT=1 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

if [ "$(id -u)" -eq 0 ]; then
  echo "Run this as your normal user (it calls sudo itself), not as root." >&2
  exit 1
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "==> workspace: $WS_DIR"

# --- 1. apt packages -------------------------------------------------------
PKGS=(
  build-essential
  git
  python3-pip
  python3-venv
  python3-serial            # pyserial, for talking to Arduino over /dev/ttyACM*
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
  ros-humble-xacro
  ros-humble-joint-state-publisher-gui
  ros-humble-tf2-tools
  ros-humble-rqt-common-plugins
)
[ "$WITH_MOVEIT" -eq 1 ] && PKGS+=(ros-humble-moveit)

echo "==> installing apt packages"
sudo apt-get update
sudo apt-get install -y "${PKGS[@]}"

# --- 2. rosdep -------------------------------------------------------------
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "==> rosdep init"
  sudo rosdep init
fi
echo "==> rosdep update"
rosdep update

# --- 3. serial port access (Arduino / MELFA over USB) ----------------------
if ! id -nG "$USER" | grep -qw dialout; then
  echo "==> adding $USER to the dialout group (log out/in for it to take effect)"
  sudo usermod -aG dialout "$USER"
fi

# --- 4. shell setup --------------------------------------------------------
MARKER="# >>> ros2_ws env (managed by scripts/setup_env.sh) >>>"
if ! grep -qF "$MARKER" "$HOME/.bashrc"; then
  echo "==> appending workspace env block to ~/.bashrc"
  cat >> "$HOME/.bashrc" <<BASHRC

$MARKER
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
# <<< ros2_ws env <<<
BASHRC
fi

# --- 5. resolve declared dependencies and build ----------------------------
echo "==> resolving package dependencies with rosdep"
source /opt/ros/humble/setup.bash
rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y || \
  echo "!! rosdep reported unresolved deps; see output above"

echo
echo "Done. Next:"
echo "  source /opt/ros/humble/setup.bash"
echo "  cd $WS_DIR && colcon build --symlink-install"
echo "  source install/setup.bash"
