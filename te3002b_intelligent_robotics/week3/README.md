# Week 3 - Mini Challenge 2 (PID Closed-Loop Control)

This folder contains the Week 3 project for Mini Challenge 2.

The goal is to move the Puzzlebot through target poses using closed-loop control (P, PI, or PID), with robust behavior in simulation and on the real robot.

## Folder Overview

- `week3_puzzlebot_interfaces/`: custom ROS 2 messages
- `week3_puzzlebot_pid/`: controller/path-planning logic package

## What Is the Same as Week 2

- Uses ROS 2 nodes and launch-based execution.
- Uses `colcon build` and workspace sourcing (`install/setup.*`).
- Uses the same robot command interface (`/cmd_vel`) and puzzlebot workflow (Gazebo + real robot).
- Still organizes behavior into modular nodes (generator + controller style architecture).

## What Is Different from Week 2

- **Control strategy:**
  - Week 2 focused on open-loop time-based motion.
  - Week 3 focuses on **closed-loop PID pose control** using feedback from robot state.

- **Message architecture:**
  - Week 2 used standard messages for simple waypoint handoff.
  - Week 3 adds a **custom `/goals` message** in `week3_puzzlebot_interfaces/msg/GoalPair.msg`.

- **Challenge requirements:**
  - Week 2 required basic trajectory execution.
  - Week 3 requires robustness against noise, perturbations, and nonlinearities, plus reachability awareness.

- **Target path specification:**
  - Week 2 examples used short waypoint sets.
  - Week 3 requires a square path of side length 2 m, starting at `(x, y, theta) = (0, 0, 0)`.

## Packages

### 1) `week3_puzzlebot_interfaces`

Defines the custom message used for `/goals`:

- `msg/GoalPair.msg`
  - `current_goal` (`geometry_msgs/Pose`)
  - `next_goal` (`geometry_msgs/Pose`)
  - helper fields (`has_next`, `suggested_time`, `reachable`, `note`)

### 2) `week3_puzzlebot_pid`

Python package intended for:

- PID-based pose controller node
- path generator node that publishes current/next goals and advances when a goal is reached
- launch and parameter/config integration

## Build

From this `week3` folder:

```bash
colcon build --packages-select week3_puzzlebot_interfaces week3_puzzlebot_pid
```

## Source Environment

After building (every new terminal):

```bash
source install/setup.bash
```

If you use PowerShell:

```powershell
.\install\setup.ps1
```

## Expected Runtime Topics (Design Intent)

- `/cmd_vel` (`geometry_msgs/Twist`): control command output
- `/goals` (`week3_puzzlebot_interfaces/msg/GoalPair`): current/next target pair
- `/goal_reached` (`std_msgs/Bool`): controller acknowledgment to advance goals

## Robustness Definition Used in Week 3

In this project, robustness means the controller can still achieve goal tracking safely and with bounded error under:

- sensor noise,
- actuator saturation,
- model mismatch/nonlinear behavior,
- small disturbances.

Typical implementation strategies include gain tuning, anti-windup, velocity saturation, safe stop conditions, and tolerance-based goal completion.

## Notes

- This folder currently contains the Week 3 project scaffold.
- Build artifacts (`build/`, `install/`, `log/`) will appear after `colcon build`.
