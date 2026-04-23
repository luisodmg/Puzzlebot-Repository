#!/usr/bin/env python3
"""
waypoint_follower.py
=====================
Open-loop controller that receives waypoints from path_generator and drives
the Puzzlebot to each one sequentially.

Protocol:
  - Subscribes to /waypoint  (Float32MultiArray: [x, y, desired_time])
  - Publishes   to /cmd_vel  (Twist)
  - Publishes   to /waypoint_reached (Bool: True when done)

For each waypoint the controller:
  1. Computes the required heading Δθ and distance d.
  2. Auto-tunes: derives ω and v from desired_time, OR uses default speeds.
  3. Performs reachability check (dead-zone / saturation / time feasibility).
  4. Executes FSM:  IDLE → ROTATE → SETTLE → DRIVE → SETTLE → DONE

Robustness:
  - Dead-zone avoidance   (v < MIN_LINEAR → clamped + warning)
  - Saturation guard      (v > MAX_LINEAR → clamped + warning)
  - Settling pauses       (reduces residual motion between phases)
  - Warns if time is too tight (would require saturated speed)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np

# ── Robot physical limits ─────────────────────────────────────────────────────
MIN_LINEAR   = 0.08   # m/s
MAX_LINEAR   = 0.40   # m/s
MIN_ANGULAR  = 0.15   # rad/s
MAX_ANGULAR  = 1.20   # rad/s
SETTLE_TIME  = 0.25   # s
# ──────────────────────────────────────────────────────────────────────────────


class WaypointFollower(Node):

    IDLE    = 0
    ROTATE  = 1
    SETTLE  = 2
    DRIVE   = 3
    DONE    = 4

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('default_linear_speed',  0.20)   # m/s
        self.declare_parameter('default_angular_speed', 0.50)   # rad/s
        self.declare_parameter('control_rate',          20.0)   # Hz

        self._v_default = self.get_parameter('default_linear_speed').value
        self._w_default = self.get_parameter('default_angular_speed').value
        rate_hz         = self.get_parameter('control_rate').value

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._cmd_pub  = self.create_publisher(Twist, 'cmd_vel', 10)
        self._ack_pub  = self.create_publisher(Bool, 'waypoint_reached', 10)
        self._sub      = self.create_subscription(
            Float32MultiArray, 'waypoint', self._waypoint_cb, 10
        )

        # ── FSM state ─────────────────────────────────────────────────────────
        self._state      = self.IDLE
        self._prev_state = self.IDLE
        self._t_state    = None

        self._v_cmd      = 0.0
        self._w_cmd      = 0.0
        self._t_rotate   = 0.0
        self._t_drive    = 0.0
        self._t_settle   = SETTLE_TIME
        self._busy       = False

        dt = 1.0 / rate_hz
        self._timer = self.create_timer(dt, self._loop)

        # Wait for clock
        self._clock_ready = False
        self.get_logger().info('WaypointFollower ready. Waiting for waypoints…')

    # ── Waypoint callback ─────────────────────────────────────────────────────

    def _waypoint_cb(self, msg: Float32MultiArray):
        if self._busy:
            self.get_logger().warn('Ignoring new waypoint: still executing previous one.')
            return

        x, y, t_desired = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        self.get_logger().info(f'[WP] Received → x={x:.2f}m  y={y:.2f}m  t={t_desired:.1f}s')

        # ── Compute heading and distance ──────────────────────────────────────
        d     = np.hypot(x, y)
        angle = np.arctan2(y, x)        # heading to waypoint

        if d < 0.01:
            self.get_logger().warn('Waypoint too close (d < 1 cm). Skipping.')
            self._ack()
            return

        # ── Auto-tune speeds ──────────────────────────────────────────────────
        if t_desired > 0.0:
            # Split time 30% rotate, 70% drive (heuristic)
            t_rot = t_desired * 0.30
            t_drv = t_desired * 0.70
            w_cmd = abs(angle) / t_rot if t_rot > 0 else self._w_default
            v_cmd = d / t_drv
            self.get_logger().info(
                f'[AUTO-TUNE] t={t_desired:.1f}s → v={v_cmd:.3f}m/s  ω={w_cmd:.3f}rad/s'
            )
        else:
            v_cmd = self._v_default
            w_cmd = self._w_default
            t_rot = abs(angle) / w_cmd
            t_drv = d / v_cmd

        # ── Reachability checks ───────────────────────────────────────────────
        feasible = True
        if v_cmd < MIN_LINEAR:
            self.get_logger().warn(
                f'[ROBUSTNESS] v={v_cmd:.3f} m/s is in the dead-zone. '
                f'Clamping to {MIN_LINEAR} m/s. Path may overshoot.'
            )
            v_cmd = MIN_LINEAR
            t_drv = d / v_cmd
        elif v_cmd > MAX_LINEAR:
            self.get_logger().error(
                f'[ROBUSTNESS] v={v_cmd:.3f} m/s exceeds saturation! '
                f'Clamping to {MAX_LINEAR} m/s. Arrival time WILL be longer.'
            )
            v_cmd = MAX_LINEAR
            t_drv = d / v_cmd
            feasible = False

        if w_cmd < MIN_ANGULAR:
            w_cmd = MIN_ANGULAR
            t_rot = abs(angle) / w_cmd
        elif w_cmd > MAX_ANGULAR:
            self.get_logger().error(
                f'[ROBUSTNESS] ω={w_cmd:.3f} rad/s exceeds saturation! '
                f'Clamping to {MAX_ANGULAR} rad/s.'
            )
            w_cmd = MAX_ANGULAR
            t_rot = abs(angle) / w_cmd
            feasible = False

        if not feasible:
            self.get_logger().warn(
                '⚠  Waypoint is NOT reachable within the requested time. '
                'Executing at maximum safe speed.'
            )

        # ── Store and begin execution ─────────────────────────────────────────
        self._v_cmd    = v_cmd
        self._w_cmd    = w_cmd * np.sign(angle) if angle != 0 else w_cmd
        self._t_rotate = t_rot
        self._t_drive  = t_drv
        self._busy     = True

        self.get_logger().info(
            f'[EXEC] d={d:.3f}m  θ={np.degrees(angle):.1f}°  '
            f't_rot={t_rot:.2f}s  t_drv={t_drv:.2f}s'
        )

        if not self._clock_ready:
            self.get_logger().warn('Clock not ready yet. Will start after clock.')
        self._transition(self.ROTATE)

    # ── FSM helpers ───────────────────────────────────────────────────────────

    def _elapsed(self):
        return (self.get_clock().now() - self._t_state).nanoseconds * 1e-9

    def _transition(self, new_state):
        self._prev_state = self._state
        self._state      = new_state
        self._t_state    = self.get_clock().now()
        names = {0:'IDLE', 1:'ROTATE', 2:'SETTLE', 3:'DRIVE', 4:'DONE'}
        self.get_logger().info(f'[FSM] → {names.get(new_state,"?")}')

    def _publish(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)

    def _ack(self):
        msg = Bool()
        msg.data = True
        self._ack_pub.publish(msg)
        self._busy  = False
        self._state = self.IDLE
        self.get_logger().info('Waypoint reached. Ack sent.')

    # ── Main control loop ─────────────────────────────────────────────────────

    def _loop(self):
        now = self.get_clock().now()

        if not self._clock_ready:
            if now.nanoseconds > 0:
                self._clock_ready = True
                self.get_logger().info('Clock ready.')
            return

        if self._state == self.IDLE:
            pass   # waiting for waypoint

        elif self._state == self.ROTATE:
            if abs(self._w_cmd) < 0.001:
                # No rotation needed
                self._transition(self.SETTLE)
            else:
                self._publish(w=self._w_cmd)
                if self._elapsed() >= self._t_rotate:
                    self._transition(self.SETTLE)

        elif self._state == self.SETTLE:
            self._publish()
            if self._elapsed() >= self._t_settle:
                if self._prev_state == self.ROTATE:
                    self._transition(self.DRIVE)
                elif self._prev_state == self.DRIVE:
                    self._transition(self.DONE)

        elif self._state == self.DRIVE:
            self._publish(v=self._v_cmd)
            if self._elapsed() >= self._t_drive:
                self._transition(self.SETTLE)

        elif self._state == self.DONE:
            self._publish()
            self._ack()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish()
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
