#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np


class TrafficLightNode(Node):

    def __init__(self):
        super().__init__('traffic_light_node')

        # =========================
        # Publisher
        # =========================
        self.state_pub = self.create_publisher(
            String,
            '/traffic_state',
            10
        )

        # =========================
        # Camera
        # =========================
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink max-buffers=1 drop=true",
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return

        self.get_logger().info("Traffic Light Node Started")

        # =========================
        # Timer
        # =========================
        self.timer = self.create_timer(0.05, self.loop)

        # =========================
        # Parameters
        # =========================
        self.min_area = 500
        self.threshold_frames = 3

        # =========================
        # State
        # =========================
        self.current_state = "RED"     # enforced output state
        self.allowed_state = "RED"     # FSM state
        self.last_detected_color = "UNKNOWN"
        self.last_state = "UNKNOWN"

        # =========================
        # Cycle definition
        # =========================
        self.expected_next = {
            "RED": "GREEN",
            "GREEN": "YELLOW",
            "YELLOW": "RED"
        }

        # =========================
        # Temporal filtering
        # =========================
        self.red_count = 0
        self.yellow_count = 0
        self.green_count = 0

    # ==========================================================
    # Color detection helper
    # ==========================================================
    def detect_color(self, mask):

        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        largest_area = 0
        largest_contour = None

        for c in contours:
            area = cv2.contourArea(c)
            if area > largest_area:
                largest_area = area
                largest_contour = c

        return largest_area, largest_contour, mask

    # ==========================================================
    # Main loop
    # ==========================================================
    def loop(self):

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received")
            return

        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # =========================
        # HSV ranges
        # =========================
        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([8, 255, 255])

        lower_red2 = np.array([172, 150, 100])
        upper_red2 = np.array([180, 255, 255])

        lower_yellow = np.array([20, 150, 120])
        upper_yellow = np.array([32, 255, 255])

        lower_green = np.array([40, 120, 120])
        upper_green = np.array([85, 255, 255])

        # =========================
        # Masks
        # =========================
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # =========================
        # Detection
        # =========================
        red_area, red_contour, _ = self.detect_color(red_mask)
        yellow_area, yellow_contour, _ = self.detect_color(yellow_mask)
        green_area, green_contour, _ = self.detect_color(green_mask)

        detected_color = "UNKNOWN"

        if max(red_area, yellow_area, green_area) > self.min_area:
            if red_area > yellow_area and red_area > green_area:
                detected_color = "RED"
            elif yellow_area > red_area and yellow_area > green_area:
                detected_color = "YELLOW"
            elif green_area > red_area and green_area > yellow_area:
                detected_color = "GREEN"

        # =========================
        # Temporal filtering
        # =========================
        if detected_color == "RED":
            self.red_count += 1
            self.yellow_count = 0
            self.green_count = 0

        elif detected_color == "YELLOW":
            self.yellow_count += 1
            self.red_count = 0
            self.green_count = 0

        elif detected_color == "GREEN":
            self.green_count += 1
            self.red_count = 0
            self.yellow_count = 0

        else:
            self.red_count = 0
            self.yellow_count = 0
            self.green_count = 0

        # =========================
        # CYCLE ENFORCEMENT FSM
        # =========================
        expected = self.expected_next[self.allowed_state]

        if detected_color == expected:

            if detected_color == "RED" and self.red_count >= self.threshold_frames:
                self.allowed_state = "RED"

            elif detected_color == "GREEN" and self.green_count >= self.threshold_frames:
                self.allowed_state = "GREEN"

            elif detected_color == "YELLOW" and self.yellow_count >= self.threshold_frames:
                self.allowed_state = "YELLOW"

        # final output state (what robot uses)
        self.current_state = self.allowed_state

        # =========================
        # Logging (only changes)
        # =========================
        if detected_color != self.last_detected_color:
            self.get_logger().info(
                f"[DETECTION] {detected_color} | "
                f"R:{red_area:.0f} Y:{yellow_area:.0f} G:{green_area:.0f}"
            )
            self.last_detected_color = detected_color

        if self.current_state != self.last_state:
            self.get_logger().info(f"[STATE] {self.current_state}")
            self.last_state = self.current_state

        # =========================
        # Publish
        # =========================
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    # ==========================================================
    # Cleanup
    # ==========================================================
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()