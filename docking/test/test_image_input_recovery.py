"""Recover stopped camera readers without restarting source or changing stamps."""

from types import SimpleNamespace
from unittest.mock import Mock

from docking.image_rectifier import DockingImageRectifier
import pytest


@pytest.mark.parametrize('case,expected', [
    ('both_stalled', True), ('image_alive', False), ('info_alive', False),
    ('startup', False), ('cooldown', False), ('exhausted', False), ('disabled', False)])
def test_only_both_stalled_inputs_trigger_bounded_reader_reconnection(monkeypatch, case, expected):
    monkeypatch.setattr('time.monotonic', lambda: 10.)
    old = [SimpleNamespace(sub=object(), callbacks={1: object()}) for _ in range(2)]
    node = SimpleNamespace(
        image_sub=old[0], info_sub=old[1], sync=object(),
        last_image_at=8., last_info_at=8., last_output_at=8.,
        last_input_reconnect_at=7., input_reconnects=0, input_stall_recovery_attempts=2,
        images_received=114, infos_received=114, pairs_received=114, outputs=32,
        selected_transport='compressed', get_logger=Mock(),
        destroy_subscription=Mock(), _connect_inputs=Mock())
    if case == 'image_alive': node.last_image_at = 9.8
    if case == 'info_alive': node.last_info_at = 9.8
    if case == 'startup': node.image_sub = None
    if case == 'cooldown': node.last_input_reconnect_at = 9.
    if case == 'exhausted': node.input_reconnects = 2
    if case == 'disabled': node.input_stall_recovery_attempts = 0
    DockingImageRectifier._report_health(node)
    assert node._connect_inputs.call_count == int(expected)
    assert node.destroy_subscription.call_count == 2*int(expected)
    assert all(bool(s.callbacks) is (not expected) for s in old)
    if expected:
        assert node.sync is None
        assert node.last_input_reconnect_at == 10.
        assert node.input_reconnects == 1


def test_real_readers_resume_same_live_publisher_after_watchdog_reconnection():
    """DDS domain must be isolated by the test runner; only image topics exist."""
    import time
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from test_image_rectifier import camera_pair

    rclpy.init(args=[])
    source = rclpy.create_node('test_camera_input_recovery')
    rectifier = DockingImageRectifier()
    rectifier.transport = 'raw'
    image_pub = source.create_publisher(Image, 'image', qos_profile_sensor_data)
    info_pub = source.create_publisher(CameraInfo, 'camera_info', qos_profile_sensor_data)
    observed = []
    source.create_subscription(Image, 'image_rect', observed.append, qos_profile_sensor_data)
    _, image, info = camera_pair()
    try:
        rectifier._select_transport()
        def pump(duration):
            deadline = time.monotonic()+duration
            while time.monotonic() < deadline:
                stamp = source.get_clock().now().to_msg()
                image.header.stamp = info.header.stamp = stamp
                image_pub.publish(image)
                info_pub.publish(info)
                rclpy.spin_once(rectifier, timeout_sec=.01)
                rclpy.spin_once(rectifier, timeout_sec=.01)
                rclpy.spin_once(source, timeout_sec=.01)
                time.sleep(.015)
        pump(2.)
        assert observed, (
            f'Healthy original image stream did not arrive: '
            f'images={rectifier.images_received}, info={rectifier.infos_received}, '
            f'pairs={rectifier.pairs_received}, outputs={rectifier.outputs}')
        before = len(observed)
        old_header = observed[-1].header.stamp
        # Simulate the trial's stopped reader inputs with a still live source.
        for subscriber in (rectifier.image_sub, rectifier.info_sub):
            rectifier.destroy_subscription(subscriber.sub)
        rectifier.last_image_at = rectifier.last_info_at = time.monotonic()-1.1
        rectifier.last_input_reconnect_at = time.monotonic()-2.1
        rectifier._report_health()
        assert rectifier.input_reconnects == 1
        pump(2.)
        assert len(observed) > before
        assert observed[-1].header.stamp.sec*10**9+observed[-1].header.stamp.nanosec > (
            old_header.sec*10**9+old_header.nanosec)
        assert rectifier.selected_transport == 'raw'
    finally:
        rectifier.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
