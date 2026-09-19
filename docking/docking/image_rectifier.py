"""Timestamp-preserving rectification without image_transport plugin teardown."""

import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from message_filters import Subscriber, TimeSynchronizer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class ImageRectifier:
    def __init__(self):
        self.bridge = CvBridge()
        self._key = None
        self._maps = None
        self._projection = None

    def rectify(self, image: Image, info: CameraInfo) -> tuple[Image, CameraInfo]:
        if image.header != info.header:
            raise ValueError('Image and CameraInfo must have identical stamps and frames')
        if min(image.width, image.height, info.width, info.height) <= 0:
            raise ValueError('Image dimensions must be positive')
        if (
                info.k[0] <= 0.0 or info.k[4] <= 0.0
                or info.p[0] <= 0.0 or info.p[5] <= 0.0
                or not all(math.isfinite(value) for value in (
                    *info.k, *info.d, *info.r, *info.p))):
            raise ValueError('CameraInfo has invalid calibration')
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial', 'equidistant'):
            raise ValueError(f'Unsupported distortion model: {info.distortion_model}')
        key = (
            image.width, image.height, info.width, info.height, info.distortion_model,
            tuple(info.k), tuple(info.d), tuple(info.r), tuple(info.p),
            info.binning_x, info.binning_y, info.roi.x_offset, info.roi.y_offset,
            info.roi.width, info.roi.height,
        )
        if key != self._key:
            model = PinholeCameraModel()
            model.from_camera_info(info)
            if abs(np.linalg.det(model.rotation_matrix())) < 1e-6:
                raise ValueError('CameraInfo rectification matrix is singular')
            projection = model.projection_matrix()
            arguments = (
                model.intrinsic_matrix(), model.distortion_coeffs(),
                model.rotation_matrix(), projection[:, :3],
                (image.width, image.height), cv2.CV_32FC1,
            )
            if info.distortion_model == 'equidistant':
                maps = cv2.fisheye.initUndistortRectifyMap(*arguments)
            else:
                maps = cv2.initUndistortRectifyMap(*arguments)
            self._maps = maps
            self._projection = projection
            self._key = key

        pixels = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        corrected = cv2.remap(pixels, *self._maps, interpolation=cv2.INTER_LINEAR)
        output = self.bridge.cv2_to_imgmsg(corrected, encoding=image.encoding)
        output.header = image.header
        # Publish the calibration describing the rectified pixels, including
        # ROI/binning adjustments, with the exact same acquisition timestamp.
        output_info = CameraInfo()
        output_info.header = image.header
        output_info.width, output_info.height = image.width, image.height
        output_info.distortion_model = 'plumb_bob'
        output_info.d = [0.0] * 5
        output_info.k = self._projection[:, :3].reshape(-1).tolist()
        output_info.r = np.eye(3).reshape(-1).tolist()
        output_info.p = self._projection.reshape(-1).tolist()
        return output, output_info


class DockingImageRectifier(Node):
    def __init__(self):
        super().__init__('rectify')
        self.declare_parameter('input_transport', 'auto')
        self.declare_parameter('max_rate_hz', 10.0)
        self.transport = str(self.get_parameter('input_transport').value)
        self.max_rate = float(self.get_parameter('max_rate_hz').value)
        if self.transport not in ('auto', 'raw', 'compressed'):
            raise ValueError('input_transport must be auto, raw or compressed')
        if not math.isfinite(self.max_rate) or self.max_rate <= 0.0:
            raise ValueError('max_rate_hz must be positive and finite')
        self.rectifier = ImageRectifier()
        # Match AprilTag's reliable subscriber. Small images at a bounded rate
        # avoid a second full-rate RGB stream and permit DDS retransmission.
        self.image_pub = self.create_publisher(Image, 'image_rect', 5)
        self.info_pub = self.create_publisher(
            CameraInfo, 'camera_info_rect', 5)
        self.image_sub = None
        self.started_at = time.monotonic()
        self.last_output_at = 0.0
        self.images_received = self.infos_received = self.pairs_received = 0
        self.outputs = 0
        self.selected_transport = None
        self.selection_timer = self.create_timer(0.5, self._select_transport)
        self.health_timer = self.create_timer(2.0, self._report_health)

    def _select_transport(self):
        if self.image_sub is not None:
            return
        selected = self.transport
        if selected == 'auto':
            if self.count_publishers(self.resolve_topic_name('image/compressed')):
                selected = 'compressed'
            elif time.monotonic() - self.started_at >= 2.0 and self.count_publishers(self.resolve_topic_name('image')):
                selected = 'raw'
            else:
                return
        self.selected_transport = selected
        topic = 'image/compressed' if selected == 'compressed' else 'image'
        message_type = CompressedImage if selected == 'compressed' else Image
        self.image_sub = Subscriber(
            self, message_type, topic, qos_profile=qos_profile_sensor_data)
        self.info_sub = Subscriber(
            self, CameraInfo, 'camera_info', qos_profile=qos_profile_sensor_data)
        self.image_sub.registerCallback(self._on_image)
        self.info_sub.registerCallback(self._on_info)
        self.sync = TimeSynchronizer([self.image_sub, self.info_sub], queue_size=10)
        self.sync.registerCallback(self._on_pair)
        self.get_logger().info(
            f'Rectifier input={selected} ({self.resolve_topic_name(topic)}); '
            f'output=mono8, reliable, <= {self.max_rate:g}Hz')

    def _on_image(self, _):
        self.images_received += 1

    def _on_info(self, _):
        self.infos_received += 1

    def _report_health(self):
        if time.monotonic() - self.last_output_at > 1.0:
            self.get_logger().warning(
                f'No recent rectified image: transport={self.selected_transport}, '
                f'images={self.images_received}, camera_info={self.infos_received}, '
                f'exact_pairs={self.pairs_received}, outputs={self.outputs}; '
                'check camera delivery and identical acquisition timestamps')

    def _on_pair(self, image, info):
        self.pairs_received += 1
        now = time.monotonic()
        if now - self.last_output_at < 1.0 / self.max_rate:
            return
        try:
            # Drop queued old observations; never stamp them as fresh data.
            stamp = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if age > 0.5 or age < -0.05:
                self.get_logger().warning(
                    f'Dropping delayed camera pair: age={age:.3f}s',
                    throttle_duration_sec=2.0)
                return
            header = image.header
            if isinstance(image, CompressedImage):
                pixels = self.rectifier.bridge.compressed_imgmsg_to_cv2(
                    image, desired_encoding='mono8')
            else:
                pixels = self.rectifier.bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            image = self.rectifier.bridge.cv2_to_imgmsg(pixels, encoding='mono8')
            image.header = header
            output, output_info = self.rectifier.rectify(image, info)
        except (ValueError, CvBridgeError, cv2.error) as exc:
            self.get_logger().error(f'Cannot rectify camera image: {exc}', throttle_duration_sec=2.0)
            return
        self.info_pub.publish(output_info)
        self.image_pub.publish(output)
        self.last_output_at = now
        self.outputs += 1


def main(args=None):
    rclpy.init(args=args)
    cv2.setNumThreads(1)
    node = DockingImageRectifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
