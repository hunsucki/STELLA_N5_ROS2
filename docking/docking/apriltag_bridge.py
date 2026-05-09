import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
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

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        pose_topic = self.get_parameter('pose_topic').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.publisher_ = self.create_publisher(PoseStamped, pose_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f'AprilTag bridge started: {self.source_frame} <- {self.target_frame}')

    def on_timer(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, rclpy.time.Time())
        except TransformException:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.source_frame
        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.position.z = trans.transform.translation.z
        msg.pose.orientation = trans.transform.rotation

        self.publisher_.publish(msg)


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
