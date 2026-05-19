#!/usr/bin/env python3
"""Capture camera calibration images for the Puzzlebot."""

from __future__ import annotations

import os
import re
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


IMAGE_TOPICS = ("/video_frames", "/camera/image_raw")
AUTO_INTERVAL_SEC = 3.0
WINDOW_NAME = "Puzzlebot Calibration Capture"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30


def default_output_dir() -> Path:
    env_path = os.environ.get("CALIBRATION_IMAGE_DIR")
    if env_path:
        return Path(env_path).expanduser()

    ros2_ws = Path.home() / "ros2_ws"
    if ros2_ws.exists():
        return ros2_ws / "src/computer_vision/calibration_images"

    return Path.home() / "dev_ws/src/computer_vision/calibration_images"


class FrameSource:
    def read(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def release(self) -> None:
        pass


class OpenCvCameraSource(FrameSource):
    def __init__(self, camera_id: int | str) -> None:
        self.camera_id = camera_id
        self.capture = cv2.VideoCapture(camera_id)

    def is_ready(self) -> bool:
        if not self.capture.isOpened():
            return False

        ok, frame = self.capture.read()
        if ok and frame is not None:
            self._last_frame = frame
            return True

        return False

    def read(self) -> Optional[np.ndarray]:
        last_frame = getattr(self, "_last_frame", None)
        if last_frame is not None:
            self._last_frame = None
            return last_frame

        ok, frame = self.capture.read()
        if not ok or frame is None:
            return None
        return frame

    def release(self) -> None:
        self.capture.release()


class GStreamerCameraSource(OpenCvCameraSource):
    def __init__(
        self,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: int = DEFAULT_FPS,
    ) -> None:
        self.pipeline = build_gstreamer_pipeline(width, height, fps)
        self.camera_id = "nvarguscamerasrc"
        self.capture = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)


class RosImageSource(FrameSource):
    def __init__(self, topics: tuple[str, ...]) -> None:
        try:
            import rclpy
            from sensor_msgs.msg import Image
        except ImportError as exc:
            raise RuntimeError(
                "ROS2 fallback requires rclpy and sensor_msgs in the current environment."
            ) from exc

        self.rclpy = rclpy
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_topic: Optional[str] = None

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self.node = rclpy.create_node("puzzlebot_image_capture")
        self._subscriptions = [
            self.node.create_subscription(Image, topic, self._make_callback(topic), 10)
            for topic in topics
        ]

    def _make_callback(self, topic: str):
        def callback(msg) -> None:
            frame = image_msg_to_bgr(msg)
            if frame is not None:
                self.latest_frame = frame
                self.latest_topic = topic

        return callback

    def read(self) -> Optional[np.ndarray]:
        self.rclpy.spin_once(self.node, timeout_sec=0.01)
        if self.latest_frame is None:
            return None
        return self.latest_frame.copy()

    def release(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy:
            self.rclpy.shutdown()


def build_gstreamer_pipeline(
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
    fps: int = DEFAULT_FPS,
) -> str:
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        f"width=(int){width}, height=(int){height}, framerate=(fraction){fps}/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=1"
    )


def image_msg_to_bgr(msg) -> Optional[np.ndarray]:
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = msg.encoding.lower()

    if height <= 0 or width <= 0:
        return None

    data = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding in ("bgr8", "rgb8"):
        channels = 3
        if data.size < height * step or step < width * channels:
            return None
        rows = data[: height * step].reshape((height, step))
        frame = rows[:, : width * channels].reshape((height, width, channels))
        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()

    if encoding in ("mono8", "8uc1"):
        if data.size < height * step or step < width:
            return None
        rows = data[: height * step].reshape((height, step))
        mono = rows[:, :width].reshape((height, width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    print(f"Unsupported ROS image encoding: {msg.encoding}", file=sys.stderr)
    return None


def open_frame_source() -> FrameSource:
    gstreamer_source = GStreamerCameraSource()
    if gstreamer_source.is_ready():
        print("Using Jetson CSI camera source: nvarguscamerasrc")
        return gstreamer_source

    gstreamer_source.release()
    print("Jetson CSI camera source unavailable: nvarguscamerasrc")

    for camera_id in (0, "/dev/video0"):
        source = OpenCvCameraSource(camera_id)
        if source.is_ready():
            print(f"Using OpenCV camera source: {camera_id}")
            return source

        source.release()
        print(f"Camera source unavailable: {camera_id}")

    print(f"Trying ROS2 image topics: {', '.join(IMAGE_TOPICS)}")
    return RosImageSource(IMAGE_TOPICS)


def next_image_index(output_dir: Path) -> int:
    pattern = re.compile(r"^calib_(\d{3,})\.jpg$")
    max_index = 0
    for path in output_dir.glob("calib_*.jpg"):
        match = pattern.match(path.name)
        if match:
            max_index = max(max_index, int(match.group(1)))
    return max_index + 1


def draw_text(
    frame: np.ndarray,
    text: str,
    origin: tuple[int, int],
    scale: float = 0.7,
    color: tuple[int, int, int] = (255, 255, 255),
    thickness: int = 2,
) -> None:
    x, y = origin
    cv2.putText(
        frame,
        text,
        (x + 1, y + 1),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_hud(
    frame: np.ndarray,
    auto_mode: bool,
    photos_taken: int,
    countdown: Optional[float],
) -> np.ndarray:
    hud = frame.copy()
    height, width = hud.shape[:2]
    mode = "AUTO" if auto_mode else "MANUAL"
    mode_color = (80, 220, 255) if auto_mode else (80, 255, 120)

    cv2.rectangle(hud, (0, 0), (width, 118), (0, 0, 0), -1)
    cv2.addWeighted(hud, 0.72, frame, 0.28, 0, frame)

    draw_text(frame, f"Mode: {mode}", (18, 34), 0.85, mode_color)
    draw_text(frame, f"Photos: {photos_taken}", (18, 70), 0.7)

    if auto_mode and countdown is not None:
        instruction = f"AUTO MODE: next capture in {max(0.0, countdown):.1f}s"
        draw_text(frame, instruction, (18, 104), 0.7, (80, 220, 255))
    else:
        draw_text(
            frame,
            "MANUAL MODE: press SPACE to take a single photo",
            (18, 104),
            0.7,
        )

    bottom = height - 16
    cv2.rectangle(frame, (0, height - 48), (width, height), (0, 0, 0), -1)
    draw_text(
        frame,
        "SPACE: capture | A: toggle auto every 3s | Q: quit",
        (18, bottom),
        0.55,
        (255, 255, 255),
        1,
    )

    return frame


def apply_flash(frame: np.ndarray, flash_until: float) -> np.ndarray:
    remaining = flash_until - time.monotonic()
    if remaining <= 0:
        return frame

    flash = np.full_like(frame, 255)
    alpha = min(0.65, remaining / 0.25)
    return cv2.addWeighted(flash, alpha, frame, 1.0 - alpha, 0)


def save_frame(frame: np.ndarray, output_dir: Path, image_index: int, session_count: int) -> Path:
    path = output_dir / f"calib_{image_index:03d}.jpg"
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"Failed to save image: {path}")

    print(f"Saved {path} (session: {session_count}, total: {image_index})")
    return path


def main() -> int:
    output_dir = default_output_dir()
    output_dir.mkdir(parents=True, exist_ok=True)
    image_index = next_image_index(output_dir)
    photos_taken = 0
    auto_mode = False
    next_auto_capture = time.monotonic() + AUTO_INTERVAL_SEC
    flash_until = 0.0

    try:
        source = open_frame_source()
    except RuntimeError as exc:
        print(f"Unable to open camera source: {exc}", file=sys.stderr)
        return 1

    print(f"Saving calibration images to: {output_dir}")
    print("Controls: SPACE=capture, A=toggle auto, Q=quit")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = source.read()
            if frame is None:
                key = cv2.waitKey(10) & 0xFF
                if key in (ord("q"), ord("Q")):
                    break
                continue

            now = time.monotonic()
            capture_requested = False
            if auto_mode and now >= next_auto_capture:
                capture_requested = True
                next_auto_capture = now + AUTO_INTERVAL_SEC

            if capture_requested:
                photos_taken += 1
                save_frame(frame.copy(), output_dir, image_index, photos_taken)
                image_index += 1
                flash_until = time.monotonic() + 0.25

            countdown = next_auto_capture - time.monotonic() if auto_mode else None
            display = draw_hud(frame.copy(), auto_mode, photos_taken, countdown)
            display = apply_flash(display, flash_until)
            cv2.imshow(WINDOW_NAME, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                photos_taken += 1
                save_frame(frame.copy(), output_dir, image_index, photos_taken)
                image_index += 1
                flash_until = time.monotonic() + 0.25
            elif key in (ord("a"), ord("A")):
                auto_mode = not auto_mode
                next_auto_capture = time.monotonic() + AUTO_INTERVAL_SEC
                state = "AUTO" if auto_mode else "MANUAL"
                print(f"Switched to {state} mode")
            elif key in (ord("q"), ord("Q")):
                break

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        source.release()
        cv2.destroyAllWindows()

    print(f"Capture finished. Photos taken this session: {photos_taken}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
