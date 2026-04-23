# Puzzlebot Workspace

This repository contains a ROS 2 workspace package for Puzzlebot launch configurations used in:

- Gazebo simulation scenarios
- Jetson camera + ArUco detection workflows
- micro-ROS agent startup
- Combined go-to-point + Kalman + marker publishing orchestration

## Workspace Layout

```text
Puzzlebot-Repository/
├── src/
│   └── puzzlebot_ros/
│       ├── *.launch.py
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── puzzlebot_ros/
│       │   └── __init__.py
│       ├── resource/
│       │   └── puzzlebot_ros
│       └── test/
│           ├── test_flake8.py
│           ├── test_pep257.py
│           └── test_copyright.py
└── README.md
```

## ROS 2 Package

- Package name: `puzzlebot_ros`
- Build type: `ament_python`
- Version: `0.0.0`

## Prerequisites

Install ROS 2 (recommended on Ubuntu) and make sure `colcon` is available.

In addition to ROS 2 core packages, these launch files rely on external packages/tools:

- `ros_deep_learning` (for `video_source`)
- `camera_info_publisher`
- `aruco_ros`
- `ros_gz_bridge`
- `tf2_ros`
- `micro_ros_agent`
- Gazebo Sim command line (`gz sim`)

Also ensure your environment provides any required custom executables referenced by launch files, such as `goto_point` and `kalman`.

## Build

From the workspace root (`Puzzlebot-Repository`):

```bash
colcon build --packages-select puzzlebot_ros
```

Then source your workspace:

```bash
source install/setup.bash
```

If you use `zsh`, PowerShell, or another shell, source the matching setup script for your shell.

## Run Launch Files

After sourcing the workspace, use:

```bash
ros2 launch puzzlebot_ros <launch_file_name>.launch.py
```

Examples:

```bash
ros2 launch puzzlebot_ros gazebo_empty.launch.py
ros2 launch puzzlebot_ros gazebo_box.launch.py
ros2 launch puzzlebot_ros gazebo_aruco.launch.py
ros2 launch puzzlebot_ros camera_jetson.launch.py
ros2 launch puzzlebot_ros aruco_jetson.launch.py
ros2 launch puzzlebot_ros goto_kalman.launch.py
ros2 launch puzzlebot_ros micro_ros_agent.launch.py
ros2 launch puzzlebot_ros marker_publisher.launch.py
```

## Launch File Summary

- `gazebo_empty.launch.py`: Starts `gz sim world_empty.sdf`, bridges camera/control/encoder topics, and publishes static camera transform.
- `gazebo_box.launch.py`: Same as above, using `world_box.sdf`.
- `gazebo_aruco.launch.py`: Same as above, using `world_aruco_mip.sdf`.
- `camera_jetson.launch.py`: Starts CSI camera stream (`video_source`) and camera info publisher.
- `aruco_jetson.launch.py`: Starts camera stream + camera info + ArUco marker publisher.
- `marker_publisher.launch.py`: Launches ArUco marker publisher with configurable arguments.
- `goto_kalman.launch.py`: Launches `goto_point`, `kalman`, and includes marker publisher launch.
- `micro_ros_agent.launch.py`: Launches micro-ROS agent over serial (`/dev/ttyUSB0`).

## Marker Publisher Arguments

`marker_publisher.launch.py` supports:

- `marker_size` (default: `0.1`)
- `side` (default: `left`, choices: `left|right`)
- `reference_frame` (default: `base`)

Example:

```bash
ros2 launch puzzlebot_ros marker_publisher.launch.py marker_size:=0.12 side:=right reference_frame:=base
```

## Code Quality Tests

The package includes standard ROS 2 linter tests:

```bash
colcon test --packages-select puzzlebot_ros
colcon test-result --verbose
```

Configured checks include:

- Flake8
- PEP257
- Copyright check (currently skipped in test)

## Notes

- `package.xml` and `setup.py` still contain placeholder fields for description/license.
- If running on Jetson hardware, verify camera calibration file path:
  - `file:///home/puzzlebot/.ros/jetson_cam.yaml`
- If running micro-ROS, update serial device path in the launch file if needed.

## Next Improvements

- Add missing runtime dependencies to `package.xml` (`exec_depend` entries).
- Add Python entry points in `setup.py` for any package-provided nodes.
- Replace placeholder package metadata (description/license/versioning strategy).
