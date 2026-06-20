# fm-teleop

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Teleop layer for First Motive's ROS2 stack. Groups every teleop input — device,
leader arm, VR, and vision — behind one command contract, plus the Foxglove
operator panel.

Part of First Motive's ROS2 (Humble) stack. Builds standalone here; assembled
with the other six package repos by
[`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Role |
|---------|-------|------|
| `fm_teleop_core` | ament_python | Shared command contract every input publishes to |
| `fm_teleop_device` | ament_python | Gamepad / handheld device input |
| `fm_teleop_leader` | ament_python | Leader-arm input |
| `fm_teleop_vr` | ament_python | VR controller input |
| `fm_teleop_vision` | ament_python | Vision-based input |
| `fm_teleop_panel` | npm | Foxglove operator panel |
| `fm_teleop` | ament_cmake | Metapackage grouping the input packages for a single install |

## Standalone Build

Clone into a colcon workspace's `src/`, pull dependencies, then build:

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/first-motive/fm-teleop.git
vcs import < fm-teleop/fm-teleop.repos     # no git externals — deps via rosdep + npm
cd .. && colcon build --symlink-install
colcon test && colcon test-result --verbose
```

The Foxglove panel (`fm_teleop_panel`) builds with npm, not colcon — see its own
README.

## Architecture

Every input source normalizes onto one fixed command contract; the sinks in the
control stack subscribe the fixed channels. Swap an input device without touching
anything downstream.

![contract](docs/diagrams/contract.svg)

Full channel table, source list, and the vision pipeline:
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Governance

Owner-free-on-main — see [CONTRIBUTING.md](CONTRIBUTING.md) and
[`.github/CODEOWNERS`](.github/CODEOWNERS).
