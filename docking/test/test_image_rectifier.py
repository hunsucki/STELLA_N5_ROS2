import copy

import cv2
from cv_bridge import CvBridge
from docking.image_rectifier import ImageRectifier
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo


def camera_pair(distortion=0.1):
    pixels = np.zeros((48, 64, 3), dtype=np.uint8)
    pixels[8:35, 35:55] = (255, 64, 128)
    image = CvBridge().cv2_to_imgmsg(pixels, encoding='rgb8')
    image.header.frame_id = 'camera_color_optical_frame'
    image.header.stamp.sec = 10
    info = CameraInfo()
    info.header = copy.deepcopy(image.header)
    info.width, info.height = 64, 48
    info.distortion_model = 'plumb_bob'
    info.d = [distortion, 0.0, 0.0, 0.0, 0.0]
    info.k = [50.0, 0.0, 32.0, 0.0, 50.0, 24.0, 0.0, 0.0, 1.0]
    info.r = np.eye(3).reshape(-1).tolist()
    info.p = [50.0, 0.0, 32.0, 0.0, 0.0, 50.0, 24.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return pixels, image, info


def test_rectification_matches_opencv_and_preserves_acquisition_header():
    pixels, image, info = camera_pair()
    output, output_info = ImageRectifier().rectify(image, info)
    k = np.array(info.k).reshape(3, 3)
    reference = cv2.undistort(pixels, k, np.array(info.d), None, k)
    actual = CvBridge().imgmsg_to_cv2(output, desired_encoding='passthrough')
    assert np.allclose(actual, reference, atol=1)
    assert output.header == image.header == output_info.header
    assert output.encoding == 'rgb8'
    assert list(output_info.p) == list(info.p)
    assert not any(output_info.d)
    assert bytes(output.data) != bytes(image.data)


def test_zero_distortion_preserves_pixels_and_caches_maps_between_frames():
    _, image, info = camera_pair(0.0)
    rectifier = ImageRectifier()
    output, _ = rectifier.rectify(image, info)
    assert bytes(output.data) == bytes(image.data)
    maps = rectifier._maps
    image.header.stamp.sec = info.header.stamp.sec = 11
    rectifier.rectify(image, info)
    assert rectifier._maps is maps
    info.d[0] = 0.2
    rectifier.rectify(image, info)
    assert rectifier._maps is not maps


@pytest.mark.parametrize('invalid', ['stamp', 'frame', 'uncalibrated', 'nan', 'rotation'])
def test_invalid_or_unsynchronized_camera_pair_is_rejected(invalid):
    _, image, info = camera_pair()
    if invalid == 'stamp':
        info.header.stamp.sec = 9
    elif invalid == 'frame':
        info.header.frame_id = 'other_camera'
    elif invalid == 'uncalibrated':
        info.k[0] = 0.0
    elif invalid == 'nan':
        info.d[0] = float('nan')
    elif invalid == 'rotation':
        info.r = [0.0] * 9
    with pytest.raises(ValueError):
        ImageRectifier().rectify(image, info)


def test_output_calibration_includes_binning_and_roi_adjustment():
    _, image, info = camera_pair()
    info.width, info.height = 128, 96
    info.binning_x = info.binning_y = 2
    info.roi.x_offset = info.roi.y_offset = 4
    info.roi.width, info.roi.height = 128, 96
    _, output_info = ImageRectifier().rectify(image, info)
    assert output_info.width == 64 and output_info.height == 48
    assert output_info.k[0] == 25.0
    assert output_info.k[2] == 14.0
    assert output_info.p[6] == 10.0


@pytest.mark.parametrize('compressed', [False, True])
def test_node_outputs_bounded_rate_reliable_pipeline_mono_with_original_stamp(monkeypatch, compressed):
    from types import SimpleNamespace
    from unittest.mock import Mock
    import time
    from docking.image_rectifier import DockingImageRectifier

    _, image, info = camera_pair()
    if compressed:
        pixels = CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8')
        message = CvBridge().cv2_to_compressed_imgmsg(pixels)
        message.header = image.header
        image = message
    wall = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: wall[0])
    node = SimpleNamespace(
        rectifier=ImageRectifier(), pairs_received=0, outputs=0,
        max_rate=10.0, last_output_at=0.0,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
        get_logger=Mock(), info_pub=Mock(), image_pub=Mock(),
    )
    DockingImageRectifier._on_pair(node, image, info)
    output = node.image_pub.publish.call_args.args[0]
    assert output.encoding == 'mono8'
    assert len(output.data) == info.width * info.height
    assert output.header == info.header
    assert node.info_pub.publish.call_args.args[0].header == info.header
    wall[0] += 0.03
    DockingImageRectifier._on_pair(node, image, info)
    assert node.outputs == 1
    wall[0] += 0.11
    # A late image/info pair must not refresh the output stream.
    image.header.stamp.sec = info.header.stamp.sec = 8
    DockingImageRectifier._on_pair(node, image, info)
    assert node.outputs == 1
    node.get_logger().warning.assert_called_once()
