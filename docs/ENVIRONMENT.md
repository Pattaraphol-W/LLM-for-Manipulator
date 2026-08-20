# Development Environment

Notes for setting up and understanding the dev machine for this project.
Surveyed 2026-08-20 on `MSI-MarkToMars`.

## Machine

| | |
|---|---|
| OS | Ubuntu 22.04.5 LTS under WSL2 (kernel 6.18.33.1-microsoft-standard-WSL2) |
| Windows host user | `Marktomars` (`/mnt/c/Users/Marktomars`) |
| CPU / RAM / disk | 16 threads / 7.6 GB / 950 GB free |
| ROS | ROS 2 Humble at `/opt/ros/humble` (283 packages), already sourced from `~/.bashrc:118` |
| Python | 3.10.12, `rclpy` imports cleanly |

## What was already there vs. what was missing

Present: ROS 2 Humble core (`ros2` CLI, `rclpy`, `rviz2`, `robot_state_publisher`),
`git`, `cmake`, `npm`, VS Code, the ROS 2 apt repo (`/etc/apt/sources.list.d/ros2.list`)
with its keyring.

Missing at survey time — **these are what `scripts/setup_env.sh` installs**:
`python3-pip`, `python3-venv`, `python3-colcon-common-extensions`, `python3-rosdep`,
`python3-vcstool`, `ros-humble-xacro`, `joint-state-publisher-gui`, tf2/rqt tooling.
Without colcon and rosdep the workspace cannot be built at all.

MoveIt 2, Gazebo, and `ros1_bridge` are **not** installed. MoveIt is available behind
`./scripts/setup_env.sh --moveit` (~1 GB). `ros1_bridge` has no Humble binary — see
[ROS 1 side](#the-ros-1-side) below.

## Setup

```bash
./scripts/setup_env.sh          # add --moveit if you need motion planning
# log out and back in once (dialout group), then:
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The script is idempotent, so re-run it whenever dependencies change.

## Workspace layout gotcha

`src/melfa_robot/` is an inherited subtree that contains **its own nested workspaces**,
and the ROS 2 one is duplicated:

```
src/melfa_robot/ros2_ws/src/keyence_plc_driver/          <- the one colcon builds
src/melfa_robot/ros2_ws/ros2_ws/src/keyence_plc_driver/  <- byte-identical copy, ignored
src/melfa_robot/ros_catkin_ws/                           <- ROS 1 (Noetic) sources, ignored
```

Two packages with the same name in one workspace makes `colcon build` fail outright, so
`COLCON_IGNORE` marker files were added to the duplicate nested workspace and to the ROS 1
catkin workspace. Leave them in place; delete one only if you deliberately want colcon to
descend there.

Also note that the subtree merge committed ~1200 build/`install`/`devel`/`log` artifacts
into git before `.gitignore` existed. They are ignored going forward but still tracked;
`git rm -r --cached` on those directories would clean history-forward if you want it.

## The ROS 1 side

`src/melfa_robot/ros_catkin_ws/` targets ROS 1 Noetic (the MELFA RV4-FL driver), which does
not coexist with Humble on Ubuntu 22.04. The upstream README's own answer is to run ROS 1 in
Docker or on a separate PC and bridge. Docker is **not** installed in this WSL instance
(`docker` not found) — install Docker Desktop on Windows with WSL integration, or
`apt install docker.io` inside WSL, before going down that path.

## Arduino UNO Q

App Lab is installed on Windows (`%LOCALAPPDATA%\arduino-app-lab`), which is the right place
for it — it is a GUI app that talks to the board over USB.

**The board is reached over ADB, not SSH.** The UNO Q is a Qualcomm Linux target and exposes
an ADB interface over USB-C (`VID_2341&PID_0078`, `MI_00`), plus a USB serial port on COM3.
App Lab ships its own adb, usable from WSL through Windows interop:

```bash
ADB="/mnt/c/Users/Marktomars/AppData/Local/Arduino15/packages/arduino/tools/adb/32.0.0/adb.exe"
"$ADB" devices        # board appears as 636494915
"$ADB" shell          # shell on the Debian side as user `arduino`
```

This needs **no usbipd and no WSL network changes** — interop reaches the Windows adb server
directly. usbipd would only be needed to get the *serial* port (COM3) into WSL as
`/dev/ttyACM0`, which is a separate concern from the Linux side.

SSH is a dead end here despite `sshd` running on the board: over Wi-Fi it shows 66% packet
loss and 657 ms RTT, and handshakes time out. The board's internet is likewise unusable at
~1.3 KB/s, so anything it needs must be downloaded on the laptop and `adb push`ed (27 MB/s).

See [`arduino_uno_q/`](../arduino_uno_q/) for the local VLM pipeline built on this.

## WSL2 networking and ROS 2 discovery

WSL is on default NAT networking (`eth0` = `172.24.48.209/20`, no `.wslconfig` present).
DDS discovery is multicast, and NAT blocks it between WSL and anything outside the Windows
host — so a ROS 2 node on the UNO Q, on the AGV PLC network, or on the ROS 1 bridge machine
**will not be discovered** from WSL out of the box.

Fix by creating `/mnt/c/Users/Marktomars/.wslconfig` with:

```ini
[wsl2]
networkingMode=mirrored
```

then `wsl --shutdown` from Windows. WSL then shares the host's interfaces and multicast
discovery works. Keep `ROS_DOMAIN_ID` consistent across every machine (the `~/.bashrc` block
the setup script adds defaults it to `0`); `ROS_LOCALHOST_ONLY=0` is set there too.
