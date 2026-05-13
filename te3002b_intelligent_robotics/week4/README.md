# Week 4 ROS 2 Nodes

This folder contains three ROS 2 Python nodes that work together to detect a traffic light, estimate the robot odometry, and drive the robot through a waypoint sequence.

## Files

### `odom_node.py`

This node estimates the robot pose from the left and right wheel encoder velocities.

It subscribes to:

- `/VelocityEncL` for the left wheel velocity
- `/VelocityEncR` for the right wheel velocity

It publishes:

- `/odom` as a `nav_msgs/Odometry` message

Internally, it uses a differential-drive model to integrate the wheel speeds over time and update `x`, `y`, and `theta`.

### `traffic_light.py`

This node reads the camera stream from the Jetson and detects the traffic light color.

It publishes:

- `/traffic_state` as a `std_msgs/String` message

The node looks for red, yellow, and green regions in HSV color space, filters the detections over several frames, and publishes the current traffic light state.

### `trafficlight_waypoint.py`

This node controls the robot motion based on the detected traffic light state and the odometry feedback.

It subscribes to:

- `/traffic_state`
- `/odom`

It publishes:

- `/cmd_vel` as a `geometry_msgs/Twist` message

The node follows a fixed sequence of move and turn actions. When the light is red it stops, when it is yellow it slows down, and when it is green it continues moving.

## What Runs Together

These three nodes depend on each other:

- `traffic_light.py` publishes the traffic light state
- `odom_node.py` publishes robot odometry
- `trafficlight_waypoint.py` uses both topics to command motion

## How To Run

Run each node in its own terminal.

1. Start the micro-ROS agent on the Jetson:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

2. Start the odometry node:

   ```bash
   python3 odom_node.py
   ```

3. Start the traffic light detection node:

   ```bash
   python3 traffic_light.py
   ```

4. Start the waypoint motion node:

   ```bash
   python3 trafficlight_waypoint.py
   ```

## Notes

- The traffic light node expects a Jetson camera pipeline through OpenCV and GStreamer.
- The odometry node uses encoder velocity topics named `/VelocityEncL` and `/VelocityEncR`.
- The motion node will not move until it receives odometry and a green traffic state.