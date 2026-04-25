# Week 3 - Puzzlebot Control and Odometry

This folder contains standalone ROS 2 Python nodes used in Week 3 for:

- Odometry estimation from wheel encoder velocities
- Closed-loop square trajectory tracking with PID control
- Waypoint-based trajectory following
- Pentagon waypoint generation

## Files

- odom_node.py
  - Subscribes to encoder angular velocities on /VelocityEncL and /VelocityEncR
  - Publishes robot odometry on /odom

- pid_square_controller.py
  - Subscribes to /odom
  - Publishes velocity commands to cmd_vel
  - Uses a state machine and PID loops to complete a square path

- pid_waypoint_follower.py
  - Subscribes to /odom and /waypoint
  - Publishes velocity commands on /cmd_vel
  - Executes incoming waypoints sequentially until trajectory completion

- trajectory_generator.py
  - Publishes PoseStamped goals on /waypoint
  - Generates a pentagon trajectory (6 points including closing point)

## Typical Topic Flow

- Square PID flow:
  - Encoder velocities -> odom_node.py -> /odom -> pid_square_controller.py -> /cmd_vel

- Waypoint flow:
  - trajectory_generator.py -> /waypoint -> pid_waypoint_follower.py
  - odom_node.py -> /odom -> pid_waypoint_follower.py
  - pid_waypoint_follower.py -> /cmd_vel

## Quick Run (Standalone Scripts)

Open separate terminals in this folder and source your ROS 2 environment first.

Linux/macOS (bash):

```bash
source /opt/ros/<distro>/setup.bash
```

Windows PowerShell:

```powershell
C:\dev\ros2\local_setup.ps1
```

Then run the nodes as Python scripts.

Terminal 1:

```bash
python3 odom_node.py
```

Terminal 2 (option A: square PID):

```bash
python3 pid_square_controller.py
```

Terminal 2 (option B: waypoint follower):

```bash
python3 pid_waypoint_follower.py
```

Terminal 3 (used with option B):

```bash
python3 trajectory_generator.py
```

## Notes

- These files are not packaged under a Week 3 ROS 2 package in this folder.
- If you want to use ros2 run, move these nodes into a proper ROS 2 package and define console scripts in setup.py.
- In pid_square_controller.py, cmd_vel is published without a leading slash, while waypoint follower publishes to /cmd_vel.
