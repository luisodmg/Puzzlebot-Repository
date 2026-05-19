#!/usr/bin/env python3
"""
Live camera undistortion node for Activity 2.7.

Opens the Puzzlebot Jetson CSI camera directly with the same GStreamer pipeline
used by the working calibration image capturer. If that direct camera source is
not available, it falls back to common ROS image topics.

cv2.undistort explanation for the activity:
  Camera calibration estimates the intrinsic matrix K and distortion
  coefficients. K describes focal lengths and optical center:

      [fx  0 cx]
  K = [ 0 fy cy]
      [ 0  0  1]

  The distortion coefficients model lens bending, especially radial distortion
  k1 and k2. cv2.undistort uses K plus those distortion coefficients to remap
  each distorted pixel back to its ideal pinhole-camera position. The result is
  an image where straight scene lines are closer to straight image lines, which
  is important before measurements or geometry-based computer vision.

Controls:
  SPACE  capture original + undistorted images
  Q      quit cleanly
"""
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def find_source_package_dir() -> Path:
    candidates = [
        Path.home() / 'ros2_ws/src/computer_vision',
        Path.home() / 'dev_ws/src/computer_vision',
        Path('/home/gnuno/dev_ws/src/computer_vision'),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


PACKAGE_DIR = find_source_package_dir()
DEFAULT_CAMERA_PARAMS = PACKAGE_DIR / 'config' / 'camera_params.npz'
DEFAULT_CAPTURES_DIR = PACKAGE_DIR / 'captures'
WINDOW_LIVE = 'Live Undistortion'
WINDOW_CAPTURE = 'Original | Undistorted'


def build_gstreamer_pipeline(width=640, height=480, fps=30):
    return (
        'nvarguscamerasrc ! '
        'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, framerate=(fraction){fps}/1 ! '
        'nvvidconv flip-method=0 ! '
        'video/x-raw, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! '
        'appsink drop=1'
    )


class LiveUndistortNode(Node):

    def __init__(self):
        super().__init__('live_undistort')

        # ── Parameters ──────────────────────────────────────────────────────────
        self.declare_parameter('camera_topics', ['/video_frames', '/camera/image_raw'])
        self.declare_parameter('camera_params_path', str(DEFAULT_CAMERA_PARAMS))
        self.declare_parameter('captures_dir', str(DEFAULT_CAPTURES_DIR))
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('use_gstreamer', True)

        self._camera_topics = list(self.get_parameter('camera_topics').value)
        self._params_path = Path(self.get_parameter('camera_params_path').value)
        self._captures_dir = Path(self.get_parameter('captures_dir').value)
        self._captures_dir.mkdir(parents=True, exist_ok=True)

        self._camera_matrix, self._dist_coeffs = self._load_camera_params(self._params_path)
        self._latest_frame = None
        self._source_label = None
        self._photos_taken = self._count_existing_captures()
        self._shutdown_requested = False
        self._cap = self._open_camera()

        self._subs = [
            self.create_subscription(
                Image,
                topic,
                self._make_image_callback(topic),
                10,
            )
            for topic in self._camera_topics
        ]

        self.get_logger().info(f'Loaded camera params: {self._params_path}')
        self.get_logger().info(f'Captures directory: {self._captures_dir}')
        if self._cap is not None:
            self.get_logger().info(f'Using camera source: {self._source_label}')
        else:
            self.get_logger().warn(
                f'Direct camera unavailable; waiting for topics: {", ".join(self._camera_topics)}'
            )

    def _open_camera(self):
        width = int(self.get_parameter('camera_width').value)
        height = int(self.get_parameter('camera_height').value)
        fps = int(self.get_parameter('camera_fps').value)

        if bool(self.get_parameter('use_gstreamer').value):
            pipeline = build_gstreamer_pipeline(width, height, fps)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self._source_label = 'nvarguscamerasrc'
                return cap
            cap.release()
            self.get_logger().warn('GStreamer CSI camera failed to open')

        for camera_id in (0, '/dev/video0'):
            cap = cv2.VideoCapture(camera_id)
            if cap.isOpened():
                self._source_label = str(camera_id)
                return cap
            cap.release()

        return None

    def _load_camera_params(self, path: Path):
        if not path.exists():
            raise FileNotFoundError(f'Camera parameters not found: {path}')

        data = np.load(str(path))
        camera_matrix = data['camera_matrix']
        dist_coeffs = data['dist_coeffs']
        return camera_matrix, dist_coeffs

    def _count_existing_captures(self) -> int:
        indices = []
        for path in self._captures_dir.glob('original_*.jpg'):
            try:
                indices.append(int(path.stem.split('_')[-1]))
            except ValueError:
                continue
        return max(indices, default=0)

    def _make_image_callback(self, topic: str):
        def callback(msg: Image) -> None:
            frame = self._image_msg_to_bgr(msg)
            if frame is None:
                return
            self._latest_frame = frame
            self._source_label = topic

        return callback

    def _image_msg_to_bgr(self, msg: Image):
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        encoding = msg.encoding.lower()

        if height <= 0 or width <= 0:
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in ('bgr8', 'rgb8'):
            channels = 3
            if data.size < height * step or step < width * channels:
                self.get_logger().warn('Received image buffer with invalid size')
                return None
            rows = data[: height * step].reshape((height, step))
            frame = rows[:, : width * channels].reshape((height, width, channels))
            if encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame.copy()

        if encoding in ('mono8', '8uc1'):
            if data.size < height * step or step < width:
                self.get_logger().warn('Received mono image buffer with invalid size')
                return None
            rows = data[: height * step].reshape((height, step))
            mono = rows[:, :width].reshape((height, width))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}')
        return None

    def _draw_overlay(self, frame):
        preview = frame.copy()
        height, width = preview.shape[:2]

        cv2.rectangle(preview, (0, height - 44), (width, height), (0, 0, 0), -1)
        text = f'SPACE = capturar | Q = salir | Fotos: {self._photos_taken}'
        if self._source_label:
            text += f' | Fuente: {self._source_label}'

        cv2.putText(
            preview,
            text,
            (16, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return preview

    def _save_capture_pair(self, frame) -> None:
        self._photos_taken += 1
        index = self._photos_taken

        # cv2.undistort applies the calibration model from camera_params.npz:
        # K maps camera coordinates to pixels, and distortion coefficients
        # remove radial/tangential lens deformation from the captured frame.
        undistorted = cv2.undistort(frame, self._camera_matrix, self._dist_coeffs)
        side_by_side = np.hstack((frame, undistorted))

        original_path = self._captures_dir / f'original_{index:03d}.jpg'
        undistorted_path = self._captures_dir / f'undistorted_{index:03d}.jpg'

        cv2.imwrite(str(original_path), frame)
        cv2.imwrite(str(undistorted_path), undistorted)
        cv2.imshow(WINDOW_CAPTURE, side_by_side)

        self.get_logger().info(
            f'Saved capture {index:03d}: {original_path.name}, {undistorted_path.name}'
        )

    def spin_once_ui(self) -> None:
        if self._cap is not None:
            ok, frame = self._cap.read()
            if ok and frame is not None:
                self._latest_frame = frame

        if self._latest_frame is None:
            blank = np.zeros((320, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                'Waiting for Jetson camera or ROS image topic...',
                (24, 165),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_LIVE, blank)
        else:
            cv2.imshow(WINDOW_LIVE, self._draw_overlay(self._latest_frame))

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            if self._latest_frame is not None:
                self._save_capture_pair(self._latest_frame.copy())
        elif key in (ord('q'), ord('Q')):
            self._shutdown_requested = True

    @property
    def shutdown_requested(self) -> bool:
        return self._shutdown_requested

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiveUndistortNode()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.01)
            node.spin_once_ui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
