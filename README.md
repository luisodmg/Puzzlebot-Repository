# Puzzlebot Workspace

This repository contains ROS 2 packages and coursework material for Puzzlebot simulation, perception, and control.

## What Is In This Repo

- `src/puzzlebot_ros`: ROS 2 Python package with launch files for Gazebo, Jetson camera, ArUco detection, and micro-ROS integration.
- `te3002b_intelligent_robotics/week2`: TE3002B Week 2 package (`week2_puzzlebot_control`) plus local colcon artifacts.
- `te3002b_intelligent_robotics/week3`: Week 3 reference scripts (odometry, PID controllers, trajectory generation).

## Key Files

### `src/puzzlebot_ros` package

- Launch files:
  - `gazebo_empty.launch.py`
  - `gazebo_box.launch.py`
  - `gazebo_aruco.launch.py`
  - `camera_jetson.launch.py`
  - `aruco_jetson.launch.py`
  - `marker_publisher.launch.py`
  - `goto_kalman.launch.py`
  - `micro_ros_agent.launch.py`
- Python nodes/modules:
  - `puzzlebot_ros/odom_node.py`
  - `puzzlebot_ros/pid_square_controller.py`
  - `puzzlebot_ros/pid_waypoint_follower.py`
  - `puzzlebot_ros/trajectory_generator.py`

### `te3002b_intelligent_robotics/week2/week2_puzzlebot_control`

- `open_loop_square.py`
- `path_generator.py`
- `waypoint_follower.py`
- `launch/mini_challenge.launch.py`

### `te3002b_intelligent_robotics/week3`

- `odom_node.py`
- `pid_square_controller.py`
- `pid_waypoint_follower.py`
- `trajectory_generator.py`

## Prerequisites

Required tools:

- ROS 2
- `colcon`

External packages/utilities used by launch files:

- `ros_deep_learning` (`video_source`)
- `camera_info_publisher`
- `aruco_ros`
- `ros_gz_bridge`
- `tf2_ros`
- `micro_ros_agent`
- Gazebo Sim CLI (`gz sim`)

If you use `goto_kalman.launch.py`, make sure `goto_point` and `kalman` executables are available in your ROS environment.

## Build

Run from repository root:

```bash
colcon build --packages-select puzzlebot_ros
```

## Source Workspace

Linux/macOS (`bash`):

```bash
source install/setup.bash
```

Linux (`zsh`):

```bash
source install/setup.zsh
```

Windows PowerShell:

```powershell
.\install\setup.ps1
```

## Run

Generic command:

```bash
ros2 launch puzzlebot_ros <launch_file>.launch.py
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

## Launch Summary

- `gazebo_empty.launch.py`: starts `gz sim world_empty.sdf` and bridges key robot/camera topics.
- `gazebo_box.launch.py`: same flow as empty world but with `world_box.sdf`.
- `gazebo_aruco.launch.py`: same flow with `world_aruco_mip.sdf`.
- `camera_jetson.launch.py`: Jetson CSI camera stream + camera info publisher.
- `aruco_jetson.launch.py`: camera stream + camera info + ArUco marker publisher.
- `marker_publisher.launch.py`: standalone marker publisher with launch arguments.
- `goto_kalman.launch.py`: starts go-to-point and Kalman nodes and includes marker publisher.
- `micro_ros_agent.launch.py`: starts serial micro-ROS agent.

## Marker Publisher Arguments

`marker_publisher.launch.py` supports:

- `marker_size` (default: `0.1`)
- `side` (default: `left`, choices: `left|right`)
- `reference_frame` (default: `base`)

Example:

```bash
ros2 launch puzzlebot_ros marker_publisher.launch.py marker_size:=0.12 side:=right reference_frame:=base
```

## Tests

```bash
colcon test --packages-select puzzlebot_ros
colcon test-result --verbose
```

Current test suite includes flake8, pep257, and copyright checks.

## Notes

- `te3002b_intelligent_robotics/week2/build`, `install`, and `log` are generated colcon artifacts.
- Jetson calibration path used by launch files may need to match your system setup.
- Serial port for micro-ROS (for example `/dev/ttyUSB0`) may need to be adjusted.
