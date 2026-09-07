"""ROS 2 service and legacy-topic capture for two SIYI RTSP cameras."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import threading
from typing import Dict, Tuple

import cv2
from inspection_interfaces.srv import (
    AbortCaptureRun,
    CapturePair,
    FinishCaptureRun,
    StartCaptureRun,
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from .storage import DatedCapturePaths


class GimbalCameraCaptureNode(Node):
    """Capture paired frames using mission services or legacy topics."""

    ZONE_ID_PATTERN = re.compile(r'[A-Za-z0-9_-]+')

    def __init__(self) -> None:
        """Declare parameters and create capture interfaces."""
        super().__init__('gimbal_camera_capture')

        self.declare_parameter('trigger_topic', '/camera/capture')
        self.declare_parameter('result_topic', '/camera/capture/result')
        self.declare_parameter('run_start_topic', '/camera/run/start')
        self.declare_parameter('run_finish_topic', '/camera/run/finish')
        self.declare_parameter('run_result_topic', '/camera/run/result')
        self.declare_parameter(
            'capture_run_start_service',
            '/camera/capture_run/start',
        )
        self.declare_parameter(
            'capture_pair_service',
            '/camera/capture_pair',
        )
        self.declare_parameter(
            'capture_run_finish_service',
            '/camera/capture_run/finish',
        )
        self.declare_parameter(
            'capture_run_abort_service',
            '/camera/capture_run/abort',
        )
        self.declare_parameter('output_directory', '~/capture')
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

        self._trigger_topic = self._string_parameter('trigger_topic')
        result_topic = self._string_parameter('result_topic')
        run_start_topic = self._string_parameter('run_start_topic')
        run_finish_topic = self._string_parameter('run_finish_topic')
        run_result_topic = self._string_parameter('run_result_topic')
        start_service = self._string_parameter(
            'capture_run_start_service'
        )
        capture_service = self._string_parameter('capture_pair_service')
        finish_service = self._string_parameter(
            'capture_run_finish_service'
        )
        abort_service = self._string_parameter(
            'capture_run_abort_service'
        )
        output_directory = Path(
            self._string_parameter('output_directory')
        )
        self._camera_urls = {
            'left': self._string_parameter('camera_1_url'),
            'right': self._string_parameter('camera_2_url'),
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
        self._service_capture_active = False
        self._shutting_down = threading.Event()

        self._result_publisher = self.create_publisher(
            String,
            result_topic,
            10,
        )
        self._run_result_publisher = self.create_publisher(
            String,
            run_result_topic,
            10,
        )
        self._trigger_subscription = self.create_subscription(
            Bool,
            self._trigger_topic,
            self._on_trigger,
            10,
        )
        self._run_start_subscription = self.create_subscription(
            Empty,
            run_start_topic,
            self._on_run_start,
            10,
        )
        self._run_finish_subscription = self.create_subscription(
            Empty,
            run_finish_topic,
            self._on_run_finish,
            10,
        )

        self._start_service = self.create_service(
            StartCaptureRun,
            start_service,
            self._start_run_service_callback,
        )
        self._capture_service = self.create_service(
            CapturePair,
            capture_service,
            self._capture_pair_service_callback,
        )
        self._finish_service = self.create_service(
            FinishCaptureRun,
            finish_service,
            self._finish_run_service_callback,
        )
        self._abort_service = self.create_service(
            AbortCaptureRun,
            abort_service,
            self._abort_run_service_callback,
        )

        self.get_logger().info(
            'Capture services ready: '
            f'{start_service}, {capture_service}, '
            f'{finish_service}, {abort_service}; '
            f'output={self._paths.base_directory}'
        )

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _capture_in_progress(self) -> bool:
        legacy_active = self._worker is not None and self._worker.is_alive()
        return legacy_active or self._service_capture_active

    def _start_run_service_callback(
        self,
        request: StartCaptureRun.Request,
        response: StartCaptureRun.Response,
    ) -> StartCaptureRun.Response:
        try:
            context = self._start_context(request)
            with self._state_lock:
                if self._capture_in_progress():
                    raise RuntimeError('capture in progress')
                run_directory, created, status = (
                    self._paths.start_mission_run(
                        datetime.now().astimezone(),
                        context,
                    )
                )
            response.success = True
            response.run_id = run_directory.name
            if created:
                response.message = 'capture run created'
            else:
                response.message = (
                    'idempotent start: existing run returned '
                    f'(status={status})'
                )
        except Exception as error:  # noqa: B902
            response.success = False
            response.message = str(error)
            response.run_id = ''
        return response

    def _capture_pair_service_callback(
        self,
        request: CapturePair.Request,
        response: CapturePair.Response,
    ) -> CapturePair.Response:
        response.run_id = request.run_id
        response.request_id = request.request_id
        claimed_capture = False
        try:
            self._validate_capture_request(request)
            with self._state_lock:
                if self._capture_in_progress():
                    raise RuntimeError('capture already in progress')

                existing = self._paths.find_capture(
                    request.run_id,
                    request.mission_id,
                    request.request_id,
                )
                if existing is not None:
                    return self._populate_capture_response(
                        response,
                        existing,
                        idempotent=True,
                    )

                self._paths.validate_active_run(
                    request.run_id,
                    request.mission_id,
                    request.zone_id,
                )
                self._service_capture_active = True
                claimed_capture = True

            captured_at = datetime.now().astimezone()
            run_directory, output_paths = self._paths.allocate(captured_at)
            files, errors, captured_at = self._capture_images(output_paths)
            success = len(files) == len(self._camera_urls) and not errors
            _, capture = self._paths.record_service_capture(
                captured_at=captured_at,
                files=files,
                errors=errors,
                success=success,
                request_id=request.request_id,
                zone_id=request.zone_id,
                requested_at=self._ros_time_to_iso(request.requested_at),
                robot_pose=self._serialize_pose(request.robot_pose),
            )
            self.get_logger().info(
                f'Capture {request.request_id} stored in {run_directory}'
            )
            return self._populate_capture_response(response, capture)
        except Exception as error:  # noqa: B902
            response.success = False
            response.message = str(error)
            return response
        finally:
            if claimed_capture:
                with self._state_lock:
                    self._service_capture_active = False

    def _finish_run_service_callback(
        self,
        request: FinishCaptureRun.Request,
        response: FinishCaptureRun.Response,
    ) -> FinishCaptureRun.Response:
        response.run_id = request.run_id
        try:
            self._validate_run_identifiers(request.run_id, request.mission_id)
            with self._state_lock:
                if self._capture_in_progress():
                    raise RuntimeError('capture in progress')
                run_directory, metadata_path, idempotent = (
                    self._paths.finish_mission_run(
                        datetime.now().astimezone(),
                        request.run_id,
                        request.mission_id,
                    )
                )
            response.success = True
            response.ready = True
            response.directory = str(run_directory)
            response.metadata_file = str(metadata_path)
            response.message = (
                'idempotent finish: run already ready'
                if idempotent
                else 'capture run completed and ready'
            )
        except Exception as error:  # noqa: B902
            response.success = False
            response.ready = False
            response.message = str(error)
        return response

    def _abort_run_service_callback(
        self,
        request: AbortCaptureRun.Request,
        response: AbortCaptureRun.Response,
    ) -> AbortCaptureRun.Response:
        response.run_id = request.run_id
        try:
            self._validate_run_identifiers(request.run_id, request.mission_id)
            with self._state_lock:
                if self._capture_in_progress():
                    raise RuntimeError('capture in progress')
                _, _, idempotent = self._paths.abort_mission_run(
                    datetime.now().astimezone(),
                    request.run_id,
                    request.mission_id,
                    request.reason or 'unspecified',
                )
            response.success = True
            response.message = (
                'idempotent abort: run already aborted'
                if idempotent
                else 'capture run aborted'
            )
        except Exception as error:  # noqa: B902
            response.success = False
            response.message = str(error)
        return response

    def _start_context(self, request: StartCaptureRun.Request) -> Dict:
        mission_id = request.mission_id.strip()
        if not mission_id:
            raise ValueError('mission_id must not be empty')
        if not request.zones:
            raise ValueError('at least one capture zone is required')

        zones = {}
        for zone in request.zones:
            zone_id = zone.id.strip()
            if (
                not zone_id
                or self.ZONE_ID_PATTERN.fullmatch(zone_id) is None
            ):
                raise ValueError(f'invalid zone id: {zone.id!r}')
            if zone_id in zones:
                raise ValueError(f'duplicate zone id: {zone_id}')
            values = (
                float(zone.min_x),
                float(zone.min_y),
                float(zone.max_x),
                float(zone.max_y),
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'non-finite coordinates: {zone_id}')
            min_x, min_y, max_x, max_y = values
            if min_x >= max_x or min_y >= max_y:
                raise ValueError(f'zone area must be positive: {zone_id}')
            zones[zone_id] = {
                'min_x': min_x,
                'min_y': min_y,
                'max_x': max_x,
                'max_y': max_y,
            }
        return {
            'mission_id': mission_id,
            'map_id': request.map_id,
            'map_frame': request.map_frame,
            'zone_revision': request.zone_revision,
            'zones': zones,
        }

    @staticmethod
    def _validate_run_identifiers(run_id: str, mission_id: str) -> None:
        if not run_id.strip():
            raise ValueError('run_id must not be empty')
        if not mission_id.strip():
            raise ValueError('mission_id must not be empty')

    def _validate_capture_request(self, request: CapturePair.Request) -> None:
        self._validate_run_identifiers(request.run_id, request.mission_id)
        if not request.request_id.strip():
            raise ValueError('request_id must not be empty')
        if not request.zone_id.strip():
            raise ValueError('zone_id must not be empty')

    def _populate_capture_response(
        self,
        response: CapturePair.Response,
        capture: Dict,
        idempotent: bool = False,
    ) -> CapturePair.Response:
        response.success = bool(capture.get('success', False))
        response.capture_id = int(capture.get('capture_id', 0))
        captured_at = datetime.fromisoformat(capture['captured_at'])
        self._set_ros_time(response.captured_at, captured_at)
        files = capture.get('files', {})
        errors = capture.get('errors', {})
        response.left_file = str(files.get('left', ''))
        response.right_file = str(files.get('right', ''))
        response.left_error = str(errors.get('left', ''))
        response.right_error = str(errors.get('right', ''))
        if idempotent:
            response.message = 'idempotent request: stored result returned'
        elif response.success:
            response.message = 'capture pair stored'
        else:
            response.message = 'capture pair incomplete'
        return response

    @staticmethod
    def _ros_time_to_iso(stamp) -> str:
        base = datetime.fromtimestamp(stamp.sec, timezone.utc)
        date_part = base.strftime('%Y-%m-%dT%H:%M:%S')
        return f'{date_part}.{stamp.nanosec:09d}+00:00'

    @staticmethod
    def _set_ros_time(message, timestamp: datetime) -> None:
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=timezone.utc)
        seconds = timestamp.timestamp()
        message.sec = int(seconds)
        message.nanosec = timestamp.microsecond * 1000

    def _serialize_pose(self, stamped_pose) -> Dict:
        pose = stamped_pose.pose.pose
        orientation = pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return {
            'frame_id': stamped_pose.header.frame_id,
            'stamp': self._ros_time_to_iso(stamped_pose.header.stamp),
            'source': 'amcl',
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
            'yaw': math.atan2(sin_yaw, cos_yaw),
            'orientation': {
                'x': float(orientation.x),
                'y': float(orientation.y),
                'z': float(orientation.z),
                'w': float(orientation.w),
            },
            'covariance': [
                float(value) for value in stamped_pose.pose.covariance
            ],
        }

    def _on_run_start(self, _message: Empty) -> None:
        timestamp = datetime.now().astimezone()
        with self._state_lock:
            if self._capture_in_progress():
                self._publish_run_result(
                    'start',
                    False,
                    error='capture in progress',
                    timestamp=timestamp,
                )
                return
            try:
                run_directory = self._paths.start_run(timestamp)
            except Exception as error:  # noqa: B902
                self._publish_run_result(
                    'start',
                    False,
                    error=str(error),
                    timestamp=timestamp,
                )
                return

        self._publish_run_result(
            'start',
            True,
            directory=str(run_directory),
            timestamp=timestamp,
        )

    def _on_run_finish(self, _message: Empty) -> None:
        timestamp = datetime.now().astimezone()
        with self._state_lock:
            if self._capture_in_progress():
                self._publish_run_result(
                    'finish',
                    False,
                    error='capture in progress',
                    timestamp=timestamp,
                )
                return
            if self._paths.active_mission_id:
                self._publish_run_result(
                    'finish',
                    False,
                    error='service-managed run requires FinishCaptureRun',
                    timestamp=timestamp,
                )
                return
            try:
                run_directory, metadata_path = self._paths.finish_run(
                    timestamp
                )
            except Exception as error:  # noqa: B902
                self._publish_run_result(
                    'finish',
                    False,
                    error=str(error),
                    timestamp=timestamp,
                )
                return

        self._publish_run_result(
            'finish',
            True,
            directory=str(run_directory),
            metadata_file=str(metadata_path),
            ready=True,
            timestamp=timestamp,
        )

    def _on_trigger(self, message: Bool) -> None:
        if not message.data or self._shutting_down.is_set():
            return

        with self._state_lock:
            if self._capture_in_progress():
                self._publish_capture_error('capture already in progress')
                return
            if self._paths.active_mission_id:
                self._publish_capture_error(
                    'service-managed run requires CapturePair'
                )
                return

            self._worker = threading.Thread(
                target=self._capture_pair_legacy,
                name='gimbal-camera-capture',
                daemon=True,
            )
            self._worker.start()

    def _publish_capture_error(self, error: str) -> None:
        self.get_logger().warning(error)
        run_directory = self._paths.active_run_directory
        self._publish_result(
            success=False,
            directory=str(run_directory) if run_directory else '',
            files={},
            errors={'request': error},
            run_id=self._paths.active_run_id,
        )

    def _capture_pair_legacy(self) -> None:
        requested_at = datetime.now().astimezone()
        files: Dict[str, str] = {}
        errors: Dict[str, str] = {}
        directory = ''
        metadata_file = ''
        run_id = ''

        try:
            run_directory, created = self._paths.ensure_run(requested_at)
            directory = str(run_directory)
            metadata_file = str(
                run_directory / self._paths.METADATA_FILENAME
            )
            run_id = run_directory.name
            if created:
                self._publish_run_result(
                    'auto_start',
                    True,
                    directory=directory,
                    timestamp=requested_at,
                )

            _, output_paths = self._paths.allocate(requested_at)
            files, errors, captured_at = self._capture_images(output_paths)
            success = len(files) == len(self._camera_urls) and not errors
            self._paths.record_capture(
                captured_at,
                files,
                errors,
                success,
            )
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
        with self._state_lock:
            self._worker = None
        self._publish_result(
            success,
            directory,
            files,
            errors,
            requested_at,
            run_id,
            metadata_file,
        )

    def _capture_images(
        self,
        output_paths: Dict[str, Path],
    ) -> Tuple[Dict[str, str], Dict[str, str], datetime]:
        files: Dict[str, str] = {}
        errors: Dict[str, str] = {}
        acquired_at = []
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
                    path, timestamp = future.result()
                    files[camera_name] = str(path)
                    acquired_at.append(timestamp)
                except Exception as error:  # noqa: B902
                    errors[camera_name] = str(error)
        captured_at = max(
            acquired_at,
            default=datetime.now().astimezone(),
        )
        return files, errors, captured_at

    def _capture_camera(
        self,
        camera_url: str,
        output_path: Path,
    ) -> Tuple[Path, datetime]:
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

            captured_at = datetime.now().astimezone()
            temporary_path = output_path.with_suffix('.tmp.jpg')
            saved = cv2.imwrite(
                str(temporary_path),
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
            )
            if not saved:
                raise RuntimeError(f'failed to write image: {output_path}')
            os.replace(temporary_path, output_path)
            return output_path, captured_at
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
        run_id: str = '',
        metadata_file: str = '',
    ) -> None:
        if timestamp is None:
            timestamp = datetime.now().astimezone()
        result = String()
        result.data = json.dumps(
            {
                'success': success,
                'timestamp': timestamp.isoformat(),
                'directory': directory,
                'run_id': run_id,
                'metadata_file': metadata_file,
                'files': files,
                'errors': errors,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self._result_publisher.publish(result)

    def _publish_run_result(
        self,
        action: str,
        success: bool,
        directory: str = '',
        metadata_file: str = '',
        error: str = '',
        ready: bool = False,
        timestamp: datetime = None,
    ) -> None:
        if timestamp is None:
            timestamp = datetime.now().astimezone()
        result = String()
        result.data = json.dumps(
            {
                'action': action,
                'success': success,
                'timestamp': timestamp.isoformat(),
                'run_id': Path(directory).name if directory else '',
                'directory': directory,
                'metadata_file': metadata_file,
                'ready': ready,
                'error': error,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self._run_result_publisher.publish(result)

    def destroy_node(self) -> bool:
        """Stop accepting work and briefly wait for a legacy capture."""
        self._shutting_down.set()
        with self._state_lock:
            worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the dual camera capture service node."""
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
