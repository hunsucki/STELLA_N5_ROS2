import math

from apriltag_msgs.msg import AprilTagDetectionArray
from docking.lidar_geometry import header_stamp_is_acceptable
from docking.tag_pose import homography_pose
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class AprilTagToPoseBridge(Node):
    def __init__(self):
        super().__init__('apriltag_to_pose_bridge')

        self.declare_parameter('target_frame', 'tag36h11:0')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('pose_topic', 'detected_dock_pose')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('transform_max_age_sec', 0.5)
        self.declare_parameter('transform_future_tolerance_sec', 0.05)
        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('tag_size', .154)
        # Kept for old command lines; changing a measurement's stamp would
        # transform it using the robot pose at the wrong time.
        self.declare_parameter('pose_stamp_delay_sec', 0.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        pose_topic = self.get_parameter('pose_topic').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.transform_max_age_sec = float(
            self.get_parameter('transform_max_age_sec').value)
        self.transform_future_tolerance_sec = float(
            self.get_parameter('transform_future_tolerance_sec').value)
        if (
                not all(math.isfinite(value) for value in (
                    publish_rate_hz, self.transform_max_age_sec,
                    self.transform_future_tolerance_sec))
                or publish_rate_hz <= 0.0
                or self.transform_max_age_sec <= 0.0
                or self.transform_future_tolerance_sec < 0.0):
            raise ValueError('Bridge rate and transform age limits are invalid')
        if self.get_parameter('pose_stamp_delay_sec').value != 0.0:
            self.get_logger().warning(
                'pose_stamp_delay_sec is deprecated and ignored; '
                'the original TF observation timestamp is preserved')
        self.last_published_stamp_nanoseconds = 0

        self.publisher_ = self.create_publisher(PoseStamped, pose_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        period = 1.0 / max(publish_rate_hz, 1.0)
        pose_source = str(self.get_parameter('pose_source').value)
        if pose_source == 'homography':
            self.tag_size = float(self.get_parameter('tag_size').value)
            if not math.isfinite(self.tag_size) or not .04 <= self.tag_size <= .30:
                raise ValueError('Invalid bridge tag size')
            self.detections_sub = Subscriber(self, AprilTagDetectionArray,
                '/apriltag/detections', qos_profile=qos_profile_sensor_data)
            self.info_sub = Subscriber(self, CameraInfo,
                '/apriltag/camera_info_rect', qos_profile=qos_profile_sensor_data)
            self.sync = TimeSynchronizer([self.detections_sub, self.info_sub], queue_size=5)
            self.sync.registerCallback(self.on_detection)
        elif pose_source == 'tf':
            self.timer = self.create_timer(period, self.on_timer)
        else:
            raise ValueError('pose_source must be tf or homography')
        self.get_logger().info(
            f'AprilTag bridge started ({pose_source}): {self.source_frame} <- {self.target_frame}')

    def on_detection(self, detections, info):
        if detections.header != info.header:
            return
        stamp_ns = detections.header.stamp.sec*1_000_000_000+detections.header.stamp.nanosec
        if not header_stamp_is_acceptable(stamp_ns, self.last_published_stamp_nanoseconds,
                self.get_clock().now().nanoseconds, int(self.transform_max_age_sec*1e9),
                int(self.transform_future_tolerance_sec*1e9)):
            return
        targets = [d for d in detections.detections
                   if f'{d.family}:{d.id}' == self.target_frame]
        if len(targets) != 1:
            return
        try:
            camera = self.tf_buffer.lookup_transform(self.source_frame, info.header.frame_id,
                rclpy.time.Time.from_msg(info.header.stamp)).transform
            pose = homography_pose(targets[0], info, self.tag_size, camera)
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(f'Tag projection unavailable: {exc}', throttle_duration_sec=2.)
            return
        msg = PoseStamped()
        msg.header.stamp = detections.header.stamp
        msg.header.frame_id = self.source_frame
        msg.pose = pose
        self.publisher_.publish(msg)
        self.last_published_stamp_nanoseconds = stamp_ns

    def on_timer(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, rclpy.time.Time())
        except TransformException:
            return

        stamp_nanoseconds = (
            int(trans.header.stamp.sec) * 1_000_000_000
            + int(trans.header.stamp.nanosec))
        if not header_stamp_is_acceptable(
                stamp_nanoseconds, self.last_published_stamp_nanoseconds,
                self.get_clock().now().nanoseconds,
                int(self.transform_max_age_sec * 1e9),
                int(self.transform_future_tolerance_sec * 1e9)):
            return

        msg = PoseStamped()
        msg.header.stamp = trans.header.stamp
        msg.header.frame_id = self.source_frame
        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.position.z = trans.transform.translation.z
        msg.pose.orientation = trans.transform.rotation

        self.publisher_.publish(msg)
        self.last_published_stamp_nanoseconds = stamp_nanoseconds


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagToPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
