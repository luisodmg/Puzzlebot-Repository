#!/usr/bin/env python3
"""
open_loop_square.py
====================
Open-loop controller that drives the Puzzlebot (differential-drive, nonholonomic)
in a square with a user-defined side length.

Auto-tuning:
    The user can provide EITHER:
      - desired_speed  (m/s) → times are computed automatically
      - desired_time   (s)   → speeds are computed automatically
    Whichever parameter is non-zero takes precedence.

Robustness strategies:
    1. Dead-zone compensation – velocities below MIN_LINEAR are clamped to MIN_LINEAR
       so the motors always enter the linear operating region.
    2. Saturation guard – velocities above MAX_LINEAR / MAX_ANGULAR are clamped.
    3. Reachability check – warns the user if the requested speed/time falls outside
       the robot's linear operating envelope.
    4. State settling time – a small pause between STRAIGHT and TURN states lets the
       robot mechanically settle, reducing residual velocity errors.
    5. Wait-for-time – waits for a non-zero clock before starting (Gazebo safe).

FSM states:
    IDLE  →  STRAIGHT  →  SETTLE  →  TURN  →  SETTLE  → (repeat 4×) →  STOP
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np


# ── Physical limits of the Puzzlebot ──────────────────────────────────────────
MIN_LINEAR   = 0.08   # m/s  – lower bound of linear operating region
MAX_LINEAR   = 0.40   # m/s  – upper bound (before saturation)
MIN_ANGULAR  = 0.15   # rad/s
MAX_ANGULAR  = 1.20   # rad/s
SETTLE_TIME  = 0.25   # s    – pause between states (robustness)
# ──────────────────────────────────────────────────────────────────────────────


class SquareController(Node):

    # ── FSM state IDs ──────────────────────────────────────────────────────────
    IDLE     = 0
    STRAIGHT = 1
    SETTLE   = 2
    TURN     = 3
    STOP     = 4

    def __init__(self):
        super().__init__('open_loop_square')

        # ── User-tunable parameters ───────────────────────────────────────────
        self.declare_parameter('side_length',   0.50)   # metres
        self.declare_parameter('desired_speed', 0.20)   # m/s  (0 → use time)
        self.declare_parameter('desired_time',  0.0)    # s    (0 → use speed)
        self.declare_parameter('num_sides',     4)      # 4 = square
        self.declare_parameter('control_rate',  20.0)   # Hz

        side      = self.get_parameter('side_length').value
        speed     = self.get_parameter('desired_speed').value
        time_req  = self.get_parameter('desired_time').value
        num_sides = self.get_parameter('num_sides').value
        rate_hz   = self.get_parameter('control_rate').value

        # ── Auto-tuning ───────────────────────────────────────────────────────
        if time_req > 0.0:
            # User gave time → compute speed
            linear_speed = side / time_req
            self.get_logger().info(f'[AUTO-TUNE] time={time_req:.2f}s → speed={linear_speed:.3f} m/s')
        else:
            linear_speed = speed
            self.get_logger().info(f'[AUTO-TUNE] speed={speed:.3f} m/s → time={side/speed:.2f}s')

        # ── Reachability / robustness check ───────────────────────────────────
        linear_speed = self._clamp_linear(linear_speed)
        turn_angle   = 2.0 * np.pi / num_sides          # 90° for a square
        angular_speed = linear_speed * 2.5              # heuristic tuning ratio
        angular_speed = self._clamp_angular(angular_speed)

        # Compute durations
        self._t_straight = side / linear_speed
        self._t_turn     = turn_angle / angular_speed
        self._t_settle   = SETTLE_TIME

        self._v_linear   = linear_speed
        self._v_angular  = angular_speed
        self._num_sides  = num_sides

        self.get_logger().info(
            f'[CONFIG] side={side:.2f}m | v={self._v_linear:.3f}m/s | '
            f'ω={self._v_angular:.3f}rad/s | '
            f't_straight={self._t_straight:.2f}s | t_turn={self._t_turn:.2f}s'
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ── FSM state ─────────────────────────────────────────────────────────
        self._state      = self.IDLE
        self._side_count = 0
        self._t_state    = None   # timestamp when current state started
        self._prev_state = None   # state we came from (for SETTLE)

        # ── Control timer ─────────────────────────────────────────────────────
        dt = 1.0 / rate_hz
        self._timer = self.create_timer(dt, self._loop)

        # Wait for clock (Gazebo)
        self._clock_ready = False
        self.get_logger().info('Waiting for ROS clock …')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _clamp_linear(self, v: float) -> float:
        if v < MIN_LINEAR:
            self.get_logger().warn(
                f'[ROBUSTNESS] Requested v={v:.3f} m/s is in the dead-zone. '
                f'Clamped to MIN={MIN_LINEAR} m/s.'
            )
            return MIN_LINEAR
        if v > MAX_LINEAR:
            self.get_logger().warn(
                f'[ROBUSTNESS] Requested v={v:.3f} m/s exceeds saturation. '
                f'Clamped to MAX={MAX_LINEAR} m/s.'
            )
            return MAX_LINEAR
        return v

    def _clamp_angular(self, w: float) -> float:
        if w < MIN_ANGULAR:
            return MIN_ANGULAR
        if w > MAX_ANGULAR:
            return MAX_ANGULAR
        return w

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._t_state).nanoseconds * 1e-9

    def _transition(self, new_state):
        self._prev_state = self._state
        self._state      = new_state
        self._t_state    = self.get_clock().now()
        self.get_logger().info(
            f'[FSM] state → {self._state_name(new_state)}  (side {self._side_count}/{self._num_sides})'
        )

    @staticmethod
    def _state_name(s):
        return {0:'IDLE', 1:'STRAIGHT', 2:'SETTLE', 3:'TURN', 4:'STOP'}.get(s, '?')

    def _publish(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self._pub.publish(msg)

    # ── Main control loop ─────────────────────────────────────────────────────

    def _loop(self):
        now = self.get_clock().now()

        # ── Wait for clock ────────────────────────────────────────────────────
        if not self._clock_ready:
            if now.nanoseconds > 0:
                self._clock_ready = True
                self._transition(self.STRAIGHT)
                self.get_logger().info('Clock ready. Starting square path.')
            return

        # ── FSM ───────────────────────────────────────────────────────────────
        if self._state == self.STRAIGHT:
            self._publish(v=self._v_linear)
            if self._elapsed() >= self._t_straight:
                self._side_count += 1
                self._transition(self.SETTLE)

        elif self._state == self.SETTLE:
            self._publish()   # stop briefly
            if self._elapsed() >= self._t_settle:
                if self._prev_state == self.STRAIGHT:
                    if self._side_count >= self._num_sides:
                        self._transition(self.TURN)
                    else:
                        self._transition(self.TURN)
                elif self._prev_state == self.TURN:
                    if self._side_count >= self._num_sides:
                        self._transition(self.STOP)
                    else:
                        self._transition(self.STRAIGHT)

        elif self._state == self.TURN:
            self._publish(w=self._v_angular)
            if self._elapsed() >= self._t_turn:
                self._transition(self.SETTLE)

        elif self._state == self.STOP:
            self._publish(v=0.0, w=0.0)
            self._publish(v=0.0, w=0.0)
            self._publish(v=0.0, w=0.0)
            self._timer.cancel()
            self.get_logger().info('Square path complete!')

        # IDLE: do nothing


def main(args=None):
    rclpy.init(args=args)
    node = SquareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish()   # safety stop
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
