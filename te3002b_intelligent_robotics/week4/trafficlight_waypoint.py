#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Helper function to keep angles between -PI and PI
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class TrafficMotionNode(Node):

    def __init__(self):
        super().__init__('traffic_motion_node')

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(String, '/traffic_state', self.state_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # =========================
        # Publisher
        # =========================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # =========================
        # State Variables
        # =========================
        self.current_state = "RED"
        self.motion_state = "STOPPED"

        # Robot current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Step initialization variables
        self.step_initialized = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        # =========================
        # Waypoint Sequence Setup
        # =========================
        distance = 0.35  # meters
        self.sequence = [
            ('MOVE', distance),
            ('TURN', math.radians(45)),
            ('MOVE', distance),
            ('TURN', math.radians(-45)),
            ('MOVE', distance),
            ('TURN', math.radians(45)),
            ('MOVE', distance),
            ('TURN', math.radians(-45)),
            ('MOVE', distance),
            ('TURN', math.radians(45)),
            ('MOVE', distance)
        ]
        self.seq_idx = 0

        # =========================
        # PID & Limits Setup
        # =========================
        # Proportional Constants (Tune these if robot is too aggressive or too slow)
        self.kp_linear = 0.8
        self.kp_angular = 1.5

        # Maximum speeds
        self.max_v = 0.15  # m/s
        self.max_w = 0.40  # rad/s

        # Tolerances (When to consider the waypoint "reached")
        self.dist_tolerance = 0.01  # 1 cm
        self.angle_tolerance = 0.03 # ~1.7 degrees

        # Timer loop (20 Hz)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("Traffic Motion Node Started - Waiting for GREEN light and ODOM.")

    def state_cb(self, msg):
        self.current_state = msg.data

    def odom_cb(self, msg):
        """Extract x, y, and yaw from the Odometry message."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion (we only care about the Z axis for planar robots)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Simplified euler conversion for 2D yaw: yaw = 2 * atan2(z, w)
        self.current_yaw = 2.0 * math.atan2(qz, qw)
        self.odom_received = True

    def loop(self):
        cmd = Twist()
        
        # Don't do anything if we haven't received sensor data yet
        if not self.odom_received:
            return

        detected = self.current_state.upper()

        # =========================
        # Decision-making layer
        # =========================
        if detected == "RED":
            self.motion_state = "STOPPED"
        elif detected == "YELLOW":
            if self.motion_state != "STOPPED":
                self.motion_state = "SLOW"
        elif detected == "GREEN":
            self.motion_state = "GO"

        # Check if sequence is finished
        if self.seq_idx >= len(self.sequence):
            self.cmd_pub.publish(cmd)
            return

        # =========================
        # PID Control Layer
        # =========================
        speed_scale = 0.0
        if self.motion_state == "GO":
            speed_scale = 1.0
        elif self.motion_state == "SLOW":
            speed_scale = 0.5 

        if speed_scale > 0.0:
            action, target = self.sequence[self.seq_idx]

            # 1. Initialize the step (take a snapshot of where we started)
            if not self.step_initialized:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_yaw = self.current_yaw
                self.step_initialized = True
                self.get_logger().info(f"Starting {action} to {target}")

            # 2. Calculate errors and velocities
            if action == 'MOVE':
                # Calculate distance traveled from start point
                dist_traveled = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
                error = target - dist_traveled

                # P-Controller logic
                desired_v = self.kp_linear * error

                # Clamp max velocity and apply traffic light scale
                current_limit = self.max_v * speed_scale
                cmd.linear.x = max(-current_limit, min(current_limit, desired_v))
                cmd.angular.z = 0.0

                # Check completion
                if abs(error) <= self.dist_tolerance:
                    self.get_logger().info("Finished MOVE")
                    self.seq_idx += 1
                    self.step_initialized = False

            elif action == 'TURN':
                # Calculate target yaw in global coordinates and normalize it
                target_yaw_global = normalize_angle(self.start_yaw + target)
                
                # Calculate shortest angular error
                error = normalize_angle(target_yaw_global - self.current_yaw)

                # P-Controller logic
                desired_w = self.kp_angular * error

                # Clamp max velocity and apply traffic light scale
                current_limit = self.max_w * speed_scale
                cmd.linear.x = 0.0
                cmd.angular.z = max(-current_limit, min(current_limit, desired_w))

                # Check completion
                if abs(error) <= self.angle_tolerance:
                    self.get_logger().info("Finished TURN")
                    self.seq_idx += 1
                    self.step_initialized = False

        else:
            # STOPPED
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()