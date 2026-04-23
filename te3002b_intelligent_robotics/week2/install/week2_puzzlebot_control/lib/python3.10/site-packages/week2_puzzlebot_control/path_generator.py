#!/usr/bin/env python3
"""
path_generator.py
==================
Publishes a sequence of waypoints for the Puzzlebot to visit.
Each waypoint is published as a Float32MultiArray:
    [x,  y,  desired_time]
where:
    x, y            – coordinates in world frame (metres, relative to start)
    desired_time    – how long the robot has to reach that waypoint (seconds)
                      0.0 means "use default speed"

The node waits for the controller to acknowledge each waypoint via the
/waypoint_reached (Bool) topic before sending the next one.

Default 3-waypoint path (can be overridden with ROS parameters):
    Start  →  P1(1.0, 0.0)  →  P2(1.0, 1.0)  →  P3(0.0, 1.0)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np


class PathGenerator(Node):

    def __init__(self):
        super().__init__('path_generator')

        # ── Declare waypoints as flat parameter list ──────────────────────────
        # Format: [x1, y1, t1,  x2, y2, t2,  x3, y3, t3]
        self.declare_parameter(
            'waypoints',
            [0.5, 0.0, 3.0,    # P1: 0.5m right in 3 s
             0.5, 0.5, 3.0,    # P2: turn and 0.5m up in 3 s
             0.0, 0.5, 4.0]    # P3: back left in 4 s
        )
        self.declare_parameter('publish_rate', 2.0)   # Hz (how often to retry)

        raw = self.get_parameter('waypoints').value
        self._waypoints = self._parse_waypoints(raw)
        self._current   = 0
        self._waiting   = False

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._pub = self.create_publisher(Float32MultiArray, 'waypoint', 10)
        self._sub = self.create_subscription(Bool, 'waypoint_reached',
                                             self._reached_cb, 10)

        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish_current)

        self.get_logger().info(
            f'PathGenerator ready. {len(self._waypoints)} waypoints loaded.'
        )
        for i, wp in enumerate(self._waypoints):
            self.get_logger().info(f'  WP{i+1}: x={wp[0]:.2f}m  y={wp[1]:.2f}m  t={wp[2]:.1f}s')

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_waypoints(raw):
        """Convert flat list [x1,y1,t1, x2,y2,t2, ...] to list of tuples."""
        wps = []
        raw = list(raw)
        if len(raw) % 3 != 0:
            raise ValueError('waypoints parameter must have length divisible by 3 (x, y, t per point).')
        for i in range(0, len(raw), 3):
            wps.append((float(raw[i]), float(raw[i+1]), float(raw[i+2])))
        return wps

    def _reached_cb(self, msg: Bool):
        if msg.data and self._waiting:
            self._waiting = False
            self._current += 1
            if self._current >= len(self._waypoints):
                self.get_logger().info('All waypoints delivered.')
                self._timer.cancel()
            else:
                self.get_logger().info(
                    f'Waypoint {self._current} reached. '
                    f'Sending WP{self._current+1}…'
                )

    def _publish_current(self):
        if self._current >= len(self._waypoints):
            return
        if self._waiting:
            return   # wait for acknowledgment

        wp = self._waypoints[self._current]
        msg = Float32MultiArray()
        msg.data = [wp[0], wp[1], wp[2]]
        self._pub.publish(msg)
        self._waiting = True
        self.get_logger().info(
            f'→ Sent WP{self._current+1}: x={wp[0]:.2f}m  y={wp[1]:.2f}m  t={wp[2]:.1f}s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
