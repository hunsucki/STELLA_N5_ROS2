"""ROS 2 topic-triggered still capture for two SIYI RTSP cameras."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import threading
from typing import Dict

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .storage import DatedCapturePaths


class GimbalCameraCaptureNode(Node):
    """Capture one frame from each configured camera after a true command."""

    def __init__(self) -> None:
        """Declare parameters and create the command/result topics."""
        super().__init__('gimbal_camera_capture')

        self.declare_parameter('trigger_topic', '/camera/capture')
        self.declare_parameter('result_topic', '/camera/capture/result')
        self.declare_parameter('output_directory', '~/capcture')
        self.declare_parameter(
            'camera_1_url',
            'rtsp://192.168.144.25:8554/main.264',
        )
        self.declare_parameter(
            'camera_2_url',
            'rtsp://192.168.144.26:8554/main.264',
        )
        self.declare_parameter('open_timeout_ms', 5000)
        self.declare_parameter('read_timeout_ms', 5000)
        self.declare_parameter('frame_read_attempts', 5)
        self.declare_parameter('jpeg_quality', 95)

        self._trigger_topic = str(
            self.get_parameter('trigger_topic').value
        )
        result_topic = str(self.get_parameter('result_topic').value)
        output_directory = Path(
            str(self.get_parameter('output_directory').value)
        )
        self._camera_urls = {
            'left': str(self.get_parameter('camera_1_url').value),
            'right': str(self.get_parameter('camera_2_url').value),
        }
        self._open_timeout_ms = int(
            self.get_parameter('open_timeout_ms').value
        )
        self._read_timeout_ms = int(
            self.get_parameter('read_timeout_ms').value
        )
        self._frame_read_attempts = max(
            1,
            int(self.get_parameter('frame_read_attempts').value),
        )
        self._jpeg_quality = min(
            100,
            max(0, int(self.get_parameter('jpeg_quality').value)),
        )

        self._paths = DatedCapturePaths(output_directory)
        self._state_lock = threading.Lock()
        self._worker = None
        self._shutting_down = threading.Event()

        self._result_publisher = self.create_publisher(String, result_topic, 10)
        self._trigger_subscription = self.create_subscription(
            Bool,
            self._trigger_topic,
            self._on_trigger,
            10,
        )

        self.get_logger().info(
            f'Waiting for Bool(true) on {self._trigger_topic}; '
            f'output={self._paths.base_directory}'
        )

    def _on_trigger(self, message: Bool) -> None:
        if not message.data or self._shutting_down.is_set():
            return

        with self._state_lock:
            if self._worker is not None and self._worker.is_alive():
                self.get_logger().warning(
                    'Capture request ignored because a capture is in progress.'
                )
                self._publish_result(
                    success=False,
                    directory='',
                    files={},
                    errors={'request': 'capture already in progress'},
                )
                return

            self._worker = threading.Thread(
                target=self._capture_pair,
                name='gimbal-camera-capture',
                daemon=True,
            )
            self._worker.start()

    def _capture_pair(self) -> None:
        timestamp = datetime.now().astimezone()
        files: Dict[str, str] = {}
        errors: Dict[str, str] = {}
        directory = ''

        try:
            date_directory, output_paths = self._paths.allocate(timestamp)
            directory = str(date_directory)
            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = {
                    camera_name: executor.submit(
                        self._capture_camera,
                        camera_url,
                        output_paths[camera_name],
                    )
                    for camera_name, camera_url in self._camera_urls.items()
                }
                for camera_name, future in futures.items():
                    try:
                        files[camera_name] = str(future.result())
                    except Exception as error:  # noqa: B902
                        errors[camera_name] = str(error)
        except Exception as error:  # noqa: B902
            errors['request'] = str(error)

        success = len(files) == len(self._camera_urls) and not errors
        if success:
            self.get_logger().info(
                f'Captured both cameras in {directory}'
            )
        else:
            self.get_logger().error(
                f'Capture incomplete: files={files}, errors={errors}'
            )
        self._publish_result(success, directory, files, errors, timestamp)

    def _capture_camera(
        self,
        camera_url: str,
        output_path: Path,
    ) -> Path:
        capture = self._open_camera(camera_url)
        try:
            if not capture.isOpened():
                raise RuntimeError(f'cannot open RTSP stream: {camera_url}')

            frame = None
            for _ in range(self._frame_read_attempts):
                received, candidate = capture.read()
                if received and candidate is not None:
                    frame = candidate

            if frame is None:
                raise RuntimeError(f'no frame received from {camera_url}')

            temporary_path = output_path.with_suffix('.tmp.jpg')
            saved = cv2.imwrite(
                str(temporary_path),
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
            )
            if not saved:
                raise RuntimeError(f'failed to write image: {output_path}')
            os.replace(temporary_path, output_path)
            return output_path
        finally:
            capture.release()

    def _open_camera(self, camera_url: str):
        parameters = [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
            self._open_timeout_ms,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC,
            self._read_timeout_ms,
        ]
        try:
            return cv2.VideoCapture(
                camera_url,
                cv2.CAP_FFMPEG,
                parameters,
            )
        except (cv2.error, TypeError):
            return cv2.VideoCapture(camera_url)

    def _publish_result(
        self,
        success: bool,
        directory: str,
        files: Dict[str, str],
        errors: Dict[str, str],
        timestamp: datetime = None,
    ) -> None:
        if timestamp is None:
            timestamp = datetime.now().astimezone()
        result = String()
        result.data = json.dumps(
            {
                'success': success,
                'timestamp': timestamp.isoformat(),
                'directory': directory,
                'files': files,
                'errors': errors,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self._result_publisher.publish(result)

    def destroy_node(self) -> bool:
        """Stop accepting work and briefly wait for an active capture."""
        self._shutting_down.set()
        with self._state_lock:
            worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the camera capture node."""
    rclpy.init(args=args)
    node = GimbalCameraCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
