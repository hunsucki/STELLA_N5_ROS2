"""Contract tests for inspection capture service callbacks."""

from datetime import datetime, timezone
import math

import pytest
import yaml


CAPTURED_AT = datetime(2026, 9, 3, 6, 36, 33, tzinfo=timezone.utc)


def make_node(tmp_path):
    """Create a capture node whose output is isolated to pytest tmp_path."""
    from gimbal_camera_capture.capture_node import GimbalCameraCaptureNode
    import rclpy

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'output_directory:={tmp_path}',
    ])
    return GimbalCameraCaptureNode()


def destroy_node(node):
    """Destroy a test node and its ROS context."""
    import rclpy

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def make_start_request(
    mission_id='mission-123',
    zone_names=('zone_1', 'greenhouse_3'),
):
    """Build a start request with the selected ordered capture zones."""
    from inspection_interfaces.msg import CaptureZone
    from inspection_interfaces.srv import StartCaptureRun

    request = StartCaptureRun.Request()
    request.mission_id = mission_id
    request.map_id = 'greenhouse-map'
    request.map_frame = 'map'
    request.zone_revision = 'zones-v1'
    for index, name in enumerate(zone_names):
        zone = CaptureZone()
        zone.id = name
        zone.min_x = float(index)
        zone.min_y = 0.0
        zone.max_x = float(index + 1)
        zone.max_y = 1.0
        request.zones.append(zone)
    return request


def make_capture_request(
    run_id,
    request_id='capture-000001',
    zone_id='zone_1',
):
    """Build a capture request containing zone and AMCL context."""
    from inspection_interfaces.srv import CapturePair

    request = CapturePair.Request()
    request.run_id = run_id
    request.mission_id = 'mission-123'
    request.request_id = request_id
    request.zone_id = zone_id
    request.requested_at.sec = 1788417393
    request.requested_at.nanosec = 100000000
    request.robot_pose.header.frame_id = 'map'
    request.robot_pose.header.stamp.sec = 1788417393
    request.robot_pose.header.stamp.nanosec = 80000000
    request.robot_pose.pose.pose.position.x = -3.2
    request.robot_pose.pose.pose.position.y = -2.1
    request.robot_pose.pose.pose.orientation.z = 0.70710678
    request.robot_pose.pose.pose.orientation.w = 0.70710678
    request.robot_pose.pose.covariance[0] = 0.25
    return request


def install_fake_capture(node):
    """Replace RTSP access with deterministic paired image files."""
    calls = []

    def capture(output_paths):
        calls.append(dict(output_paths))
        for path in output_paths.values():
            path.write_bytes(b'jpeg')
        files = {
            camera: str(path)
            for camera, path in output_paths.items()
        }
        return files, {}, CAPTURED_AT

    node._capture_images = capture
    return calls


def test_service_flow_is_synchronous_and_idempotent(tmp_path):
    """Start, capture, retry, and finish satisfy the shared contract."""
    from inspection_interfaces.srv import CapturePair, FinishCaptureRun
    from inspection_interfaces.srv import StartCaptureRun

    node = make_node(tmp_path)
    try:
        start_request = make_start_request()
        start = node._start_run_service_callback(
            start_request,
            StartCaptureRun.Response(),
        )
        retry_start = node._start_run_service_callback(
            start_request,
            StartCaptureRun.Response(),
        )
        run_directory = node._paths.active_run_directory
        calls = install_fake_capture(node)

        capture_request = make_capture_request(start.run_id)
        capture = node._capture_pair_service_callback(
            capture_request,
            CapturePair.Response(),
        )
        retry_capture = node._capture_pair_service_callback(
            capture_request,
            CapturePair.Response(),
        )

        finish_request = FinishCaptureRun.Request()
        finish_request.run_id = start.run_id
        finish_request.mission_id = start_request.mission_id
        finish = node._finish_run_service_callback(
            finish_request,
            FinishCaptureRun.Response(),
        )
        retry_finish = node._finish_run_service_callback(
            finish_request,
            FinishCaptureRun.Response(),
        )

        assert start.success is True
        assert retry_start.success is True
        assert retry_start.run_id == start.run_id
        assert len(calls) == 1
        assert capture.success is True
        assert capture.capture_id == 1
        assert capture.left_file.startswith('left/')
        assert capture.right_file.startswith('right/')
        assert retry_capture.capture_id == capture.capture_id
        assert retry_capture.left_file == capture.left_file
        assert finish.success is True
        assert finish.ready is True
        assert retry_finish.success is True
        assert retry_finish.ready is True

        assert run_directory is not None
        assert (run_directory / capture.left_file).read_bytes() == b'jpeg'
        assert (run_directory / capture.right_file).read_bytes() == b'jpeg'
        assert (run_directory / 'READY').is_file()
        with (run_directory / 'metadata.yaml').open() as stream:
            metadata = yaml.safe_load(stream)
        assert metadata['status'] == 'completed'
        assert metadata['mission_id'] == 'mission-123'
        assert metadata['captures'][0]['request_id'] == 'capture-000001'
        assert metadata['captures'][0]['zone_id'] == 'zone_1'
        assert list(metadata['zone_config']['zones']) == [
            'zone_1',
            'greenhouse_3',
        ]
        assert len(metadata['captures'][0]['robot_pose']['covariance']) == 36
    finally:
        destroy_node(node)


def test_abort_is_idempotent_and_does_not_create_ready(tmp_path):
    """Abort records its reason and remains safe to retry."""
    from inspection_interfaces.srv import AbortCaptureRun, StartCaptureRun

    node = make_node(tmp_path)
    try:
        start_request = make_start_request()
        start = node._start_run_service_callback(
            start_request,
            StartCaptureRun.Response(),
        )
        run_directory = node._paths.active_run_directory
        abort_request = AbortCaptureRun.Request()
        abort_request.run_id = start.run_id
        abort_request.mission_id = start_request.mission_id
        abort_request.reason = 'ESTOP'

        abort = node._abort_run_service_callback(
            abort_request,
            AbortCaptureRun.Response(),
        )
        retry = node._abort_run_service_callback(
            abort_request,
            AbortCaptureRun.Response(),
        )

        assert abort.success is True
        assert retry.success is True
        assert run_directory is not None
        assert not (run_directory / 'READY').exists()
        with (run_directory / 'metadata.yaml').open() as stream:
            metadata = yaml.safe_load(stream)
        assert metadata['status'] == 'aborted'
        assert metadata['abort_reason'] == 'ESTOP'
    finally:
        destroy_node(node)


@pytest.mark.parametrize(
    'zone_names',
    [
        ('zone_1',),
        ('zone_1', 'greenhouse_3'),
        tuple(f'inspection-{index}' for index in range(6)),
    ],
)
def test_start_accepts_variable_zone_counts_and_names(tmp_path, zone_names):
    """A mission accepts one, two, or more than five named zones."""
    from inspection_interfaces.srv import StartCaptureRun

    node = make_node(tmp_path)
    try:
        request = make_start_request(zone_names=zone_names)

        response = node._start_run_service_callback(
            request,
            StartCaptureRun.Response(),
        )

        assert response.success is True
        with (
            node._paths.active_run_directory / 'metadata.yaml'
        ).open() as stream:
            metadata = yaml.safe_load(stream)
        assert list(metadata['zone_config']['zones']) == list(zone_names)
    finally:
        destroy_node(node)


def test_start_rejects_empty_zone_snapshot(tmp_path):
    """A capture-enabled mission requires at least one zone."""
    from inspection_interfaces.srv import StartCaptureRun

    node = make_node(tmp_path)
    try:
        request = make_start_request(zone_names=())
        response = node._start_run_service_callback(
            request,
            StartCaptureRun.Response(),
        )

        assert response.success is False
        assert response.message == 'at least one capture zone is required'
        assert list(tmp_path.iterdir()) == []
    finally:
        destroy_node(node)


@pytest.mark.parametrize('zone_id', ['', 'bad zone', 'zone/1', '구역_1'])
def test_start_rejects_invalid_zone_names(tmp_path, zone_id):
    """Zone names only allow ASCII letters, digits, underscores, and dashes."""
    from inspection_interfaces.srv import StartCaptureRun

    node = make_node(tmp_path)
    try:
        request = make_start_request(zone_names=(zone_id,))
        response = node._start_run_service_callback(
            request,
            StartCaptureRun.Response(),
        )

        assert response.success is False
        assert response.message.startswith('invalid zone id:')
        assert list(tmp_path.iterdir()) == []
    finally:
        destroy_node(node)


def test_start_rejects_duplicate_zone_names(tmp_path):
    """A start request cannot contain a zone name twice."""
    from inspection_interfaces.srv import StartCaptureRun

    node = make_node(tmp_path)
    try:
        request = make_start_request(zone_names=('zone_1', 'zone_1'))
        response = node._start_run_service_callback(
            request,
            StartCaptureRun.Response(),
        )

        assert response.success is False
        assert response.message == 'duplicate zone id: zone_1'
        assert list(tmp_path.iterdir()) == []
    finally:
        destroy_node(node)


@pytest.mark.parametrize(
    ('field', 'value', 'expected'),
    [
        ('max_x', 0.0, 'zone area must be positive: zone_1'),
        ('min_y', math.inf, 'non-finite coordinates: zone_1'),
    ],
)
def test_start_rejects_invalid_zone_coordinates(
    tmp_path,
    field,
    value,
    expected,
):
    """Every zone must have finite coordinates and positive area."""
    from inspection_interfaces.srv import StartCaptureRun

    node = make_node(tmp_path)
    try:
        request = make_start_request(zone_names=('zone_1',))
        setattr(request.zones[0], field, value)
        response = node._start_run_service_callback(
            request,
            StartCaptureRun.Response(),
        )

        assert response.success is False
        assert response.message == expected
        assert list(tmp_path.iterdir()) == []
    finally:
        destroy_node(node)


def test_capture_rejects_zone_not_registered_at_start(tmp_path):
    """A capture request only accepts its run's start-zone snapshot."""
    from inspection_interfaces.srv import CapturePair, StartCaptureRun

    node = make_node(tmp_path)
    try:
        start = node._start_run_service_callback(
            make_start_request(zone_names=('zone_1',)),
            StartCaptureRun.Response(),
        )
        calls = install_fake_capture(node)
        request = make_capture_request(start.run_id, zone_id='unknown-zone')
        response = node._capture_pair_service_callback(
            request,
            CapturePair.Response(),
        )

        assert response.success is False
        assert response.message == 'unknown zone_id: unknown-zone'
        assert calls == []
    finally:
        destroy_node(node)
