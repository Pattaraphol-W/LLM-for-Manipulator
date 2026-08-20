# LLM for Manipulator

Research project integrating LLMs with a MELFA robot manipulator.

## Structure

- `src/` — ROS2 packages and source code
  - `src/melfa_robot/` — inherited manipulator control stack (originally from [Nattapol-M/melfa_robot](https://github.com/Nattapol-M/melfa_robot))
- `arduino_uno_q/` — on-device vision-language model on an Arduino UNO Q (see [its README](arduino_uno_q/README.md))
- `experiments/` — measurements and evaluations with raw data and runnable evaluators (see [experiments/README.md](experiments/README.md))
- `docs/` — project notes and write-ups (see [docs/README.md](docs/README.md))

## Setup

Ubuntu 22.04 + ROS 2 Humble (this machine runs it under WSL2).

```bash
./scripts/setup_env.sh          # colcon, rosdep, pip/venv, ROS tooling; --moveit for MoveIt 2
# log out and back in once (dialout group), then:
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

See [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md) for the full machine survey, the nested-workspace
gotcha in `src/melfa_robot/`, the ROS 1 (Noetic) situation, and notes on Arduino App Lab /
USB passthrough / DDS discovery under WSL2.
