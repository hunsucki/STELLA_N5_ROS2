"""Shared, fail-closed LaserScan input for docking."""

from dataclasses import dataclass
import math
import time

from docking.lidar_geometry import (
    header_stamp_is_acceptable,
    PlanarTransform,
    project_scan_to_base,
    ProjectedPoint,
    scan_is_fresh,
    scan_metadata_is_valid,
    yaw_from_quaternion,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


@dataclass(frozen=True)
class ScanSnapshot:
    scan: LaserScan
    sequence: int
    received_at: float


class DockingLidar:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('docking_lidar_topic', '/scan_2')
        node.declare_parameter('docking_lidar_frame', 'base_scan2')
        node.declare_parameter('docking_lidar_scan_max_age_sec', 0.30)
        node.declare_parameter('docking_lidar_header_max_age_sec', 0.50)
        node.declare_parameter('docking_lidar_future_tolerance_sec', 0.05)
        node.declare_parameter('docking_lidar_transform_timeout_sec', 0.20)
        node.declare_parameter('docking_lidar_wait_timeout_sec', 10.0)

    def __init__(self, node: Node, tf_buffer: Buffer) -> None:
        self.node = node
        self.tf_buffer = tf_buffer
        self.last_scan: LaserScan | None = None
        self.last_received_at = 0.0
        self.last_valid_received_at = 0.0
        self.sequence = 0
        self.last_stamp_nanoseconds = 0
        self.last_error = 'no scan received'
        self._stream_valid = False
        self._transform: PlanarTransform | None = None
        self._transform_z = 0.0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = node.create_subscription(
            LaserScan,
            node.get_parameter('docking_lidar_topic').value,
            self._scan_callback,
            sensor_qos,
        )

    @property
    def transform(self) -> PlanarTransform | None:
        return self._transform

    @property
    def transform_z(self) -> float:
        return self._transform_z

    @property
    def max_scan_age(self) -> float:
        return max(float(
            self.node.get_parameter('docking_lidar_scan_max_age_sec').value), 0.0)

    def resolve_transform(self) -> PlanarTransform | None:
        if self._transform is not None:
            return self._transform

        base_frame = self._normalize_frame(
            str(self.node.get_parameter('base_frame').value))
        lidar_frame = self.expected_frame()
        timeout = max(float(self.node.get_parameter(
            'docking_lidar_transform_timeout_sec').value), 0.0)

        try:
            stamped = self.tf_buffer.lookup_transform(
                base_frame,
                lidar_frame,
                Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException as exc:
            self.last_error = (
                f'TF {base_frame} <- {lidar_frame} unavailable: {exc}')
            return None

        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        self._transform = PlanarTransform(
            x=translation.x,
            y=translation.y,
            yaw=yaw_from_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w),
        )
        self._transform_z = translation.z
        self.last_error = ''
        return self._transform

    def validate_parameters(self) -> bool:
        topic = str(self.node.get_parameter('docking_lidar_topic').value).strip()
        frame = self.expected_frame()
        values = (
            float(self.node.get_parameter(
                'docking_lidar_scan_max_age_sec').value),
            float(self.node.get_parameter(
                'docking_lidar_header_max_age_sec').value),
            float(self.node.get_parameter(
                'docking_lidar_future_tolerance_sec').value),
            float(self.node.get_parameter(
                'docking_lidar_transform_timeout_sec').value),
            float(self.node.get_parameter(
                'docking_lidar_wait_timeout_sec').value),
        )
        if not topic or not frame or not all(math.isfinite(value) for value in values):
            self.last_error = 'docking LiDAR topic, frame, and timeouts must be valid'
            return False
        scan_age, header_age, future_tolerance, transform_timeout, wait_timeout = values
        if (
                scan_age <= 0.0
                or header_age <= 0.0
                or future_tolerance < 0.0
                or transform_timeout < 0.0
                or wait_timeout <= 0.0):
            self.last_error = 'docking LiDAR timeout parameters are outside safe bounds'
            return False
        return True

    def snapshot(self) -> ScanSnapshot | None:
        if not self._stream_valid:
            return None

        scan = self.last_scan
        if scan is None:
            self.last_error = 'no docking LiDAR scan received'
            return None

        now = time.monotonic()
        if not scan_is_fresh(self.last_received_at, now, self.max_scan_age):
            age = max(now - self.last_received_at, 0.0)
            self.last_error = f'docking LiDAR scan is stale ({age:.3f}s old)'
            return None

        actual_frame = self._normalize_frame(scan.header.frame_id)
        expected_frame = self.expected_frame()
        if actual_frame != expected_frame:
            self.last_error = (
                f'docking LiDAR frame is "{actual_frame or "<empty>"}", '
                f'expected "{expected_frame}"')
            return None

        if not scan_metadata_is_valid(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max):
            self.last_error = 'docking LiDAR scan metadata is invalid'
            return None

        if self.resolve_transform() is None:
            return None

        # Keep the scan's receipt time rather than the current time so repeatedly
        # reading one message can never make it appear fresh.
        self.last_valid_received_at = self.last_received_at
        self.last_error = ''
        return ScanSnapshot(scan, self.sequence, self.last_received_at)

    def project(
            self, snapshot: ScanSnapshot,
            sector_center: float, sector_width: float,
            min_range: float, max_range: float
            ) -> list[ProjectedPoint] | None:
        transform = self.resolve_transform()
        if transform is None:
            return None

        scan = snapshot.scan
        points = project_scan_to_base(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            transform,
            sector_center,
            sector_width,
            min_range,
            max_range,
        )
        if points is None:
            self.last_error = 'docking LiDAR scan could not be projected'
        return points

    def expected_frame(self) -> str:
        return self._normalize_frame(str(
            self.node.get_parameter('docking_lidar_frame').value))

    def _scan_callback(self, msg: LaserScan) -> None:
        stamp_nanoseconds = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec))
        now_nanoseconds = self.node.get_clock().now().nanoseconds
        max_header_age = max(float(self.node.get_parameter(
            'docking_lidar_header_max_age_sec').value), 0.0)
        future_tolerance = max(float(self.node.get_parameter(
            'docking_lidar_future_tolerance_sec').value), 0.0)
        if not header_stamp_is_acceptable(
                stamp_nanoseconds,
                self.last_stamp_nanoseconds,
                now_nanoseconds,
                int(max_header_age * 1e9),
                int(future_tolerance * 1e9)):
            self._stream_valid = False
            self.last_error = (
                'docking LiDAR header stamp is zero, duplicate, out of order, '
                'or too old')
            return

        self.last_scan = msg
        self.last_received_at = time.monotonic()
        self.last_stamp_nanoseconds = stamp_nanoseconds
        self.sequence += 1
        self._stream_valid = True

    @staticmethod
    def _normalize_frame(frame: str) -> str:
        return frame.strip().lstrip('/')
