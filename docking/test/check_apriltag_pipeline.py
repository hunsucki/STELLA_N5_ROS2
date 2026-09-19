"""Exercise rectification, AprilTag detection and bridge without robot motion.

Run after sourcing install/setup.bash:
    python3 test/check_apriltag_pipeline.py

Uses an isolated DDS domain, synthetic images and a synthetic camera TF.
Only the perception launch is started; no motion/action client is created.
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--transport', choices=('raw', 'compressed'), default='raw')
    transport = parser.parse_args().transport
    os.environ['ROS_DOMAIN_ID'] = '217'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import cv2
    import numpy as np
    import rclpy
    from apriltag_msgs.msg import AprilTagDetectionArray
    from docking.apriltag_bridge import AprilTagToPoseBridge
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    log_path = Path(tempfile.mkdtemp(prefix='docking-perception-')) / 'launch.log'
    rclpy.init()
    camera = rclpy.create_node('synthetic_docking_camera')
    bridge = AprilTagToPoseBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(bridge)
    image_pub = camera.create_publisher(
        CompressedImage if transport == 'compressed' else Image,
        '/camera/camera/color/image_raw' + ('/compressed' if transport == 'compressed' else ''),
        qos_profile_sensor_data)
    info_pub = camera.create_publisher(
        CameraInfo, '/camera/camera/color/camera_info', qos_profile_sensor_data)
    received_images, detections, poses = [], [], []
    pose_times = []

    def on_pose(msg):
        poses.append(msg)
        pose_times.append(time.monotonic())
    camera.create_subscription(
        Image, '/apriltag/image_rect', received_images.append, qos_profile_sensor_data)
    camera.create_subscription(
        AprilTagDetectionArray, '/apriltag/detections', detections.append, 10)
    camera.create_subscription(PoseStamped, '/detected_dock_pose', on_pose, 10)
    broadcaster = StaticTransformBroadcaster(camera)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_color_optical_frame'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, 0, 160)
    pixels = np.full((480, 640), 255, dtype=np.uint8)
    pixels[160:320, 360:520] = marker
    message = Image()
    message.header.frame_id = transform.child_frame_id
    message.height, message.width = pixels.shape
    message.encoding = 'rgb8'
    message.step = message.width * 3
    rgb = cv2.cvtColor(pixels, cv2.COLOR_GRAY2RGB)
    message.data = rgb.tobytes()
    info = CameraInfo()
    info.width, info.height = message.width, message.height
    info.distortion_model = 'plumb_bob'
    info.d = [0.12, -0.02, 0.0, 0.0, 0.0]
    info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    if transport == 'compressed':
        compressed = CompressedImage()
        compressed.header = message.header
        compressed.format = 'rgb8; jpeg compressed bgr8'
        compressed.data = cv2.imencode('.jpg', rgb)[1].tobytes()
        message = compressed
    published_stamps = set()

    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'docking', 'apriltag_36h11.launch.py'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20.0
            next_publish = 0.0
            steady_start = None
            while time.monotonic() < deadline:
                if len(poses) >= 5 and steady_start is None:
                    steady_start = time.monotonic()
                if steady_start is not None and time.monotonic() - steady_start >= 5.0:
                    break
                assert process.poll() is None, f'Perception launch failed: {log_path}'
                if time.monotonic() >= next_publish:
                    message.header.stamp = camera.get_clock().now().to_msg()
                    published_stamps.add((message.header.stamp.sec, message.header.stamp.nanosec))
                    info.header = message.header
                    info_pub.publish(info)
                    image_pub.publish(message)
                    next_publish = time.monotonic() + 1.0 / 30.0
                executor.spin_once(timeout_sec=0.02)

            assert steady_start is not None, f'No bridge poses; inspect {log_path}'
            steady_times = [t for t in pose_times if t >= steady_start]
            assert len(steady_times) >= 20, f'Low detection throughput: {len(steady_times)}'
            gaps = [b - a for a, b in zip(steady_times, steady_times[1:])]
            assert max(gaps) < 0.5, f'Detection gap: {max(gaps):.3f}s'
            assert received_images[-1].encoding == 'mono8'
            assert any(msg.detections for msg in detections), 'No AprilTag detection'
            assert received_images, 'No rectified images'
            assert bytes(received_images[-1].data) != pixels.tobytes(), 'No rectification'
            assert all(
                (pose.header.stamp.sec, pose.header.stamp.nanosec) in published_stamps
                for pose in poses), 'Bridge changed camera observation timestamps'

            # Let pending detections drain, then require silence after tag loss.
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            count = len(poses)
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            assert len(poses) == count, 'Cached TF was republished after camera stopped'
        finally:
            if process.poll() is None:
                # ros2 launch forwards SIGINT to its children itself. Signaling
                # the entire group would deliver it to each child twice.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.0)
            executor.shutdown()
            bridge.destroy_node()
            camera.destroy_node()
            rclpy.shutdown()

    # ros2 launch can exit 0 even when one of its children segfaults at shutdown.
    # The earlier smoke check stopped at image/pose delivery and missed this.
    output = log_path.read_text()
    assert f'Rectifier input={transport}' in output, f'Wrong auto-selected transport: {log_path}'
    assert process.returncode == 0, f'Launch did not shut down cleanly: {log_path}'
    assert 'process has died' not in output, f'Perception process crashed: {log_path}'
    assert output.count('process has finished cleanly') >= 2, (
        f'Both perception processes must exit normally: {log_path}')
    print(
        f'PASS ({transport}, 640x480 RGB at 30Hz): {len(received_images)} rectified images, '
        f'{count} poses with original timestamps; tag-loss silence and clean '
        f'shutdown verified; max steady detection gap={max(gaps):.3f}s. Log: {log_path}')


if __name__ == '__main__':
    main()
