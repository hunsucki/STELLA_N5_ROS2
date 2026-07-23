from types import SimpleNamespace

from docking.docking_lidar import DockingLidar
from docking.lidar_geometry import PlanarTransform
from sensor_msgs.msg import LaserScan


class FakeClock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)


class FakeNode:
    def __init__(self):
        self.clock = FakeClock(10_000_000_000)
        self.parameters = {
            'docking_lidar_frame': 'base_scan2',
            'docking_lidar_scan_max_age_sec': 0.30,
            'docking_lidar_header_max_age_sec': 0.50,
            'docking_lidar_future_tolerance_sec': 0.05,
        }

    def get_clock(self):
        return self.clock

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])


def make_lidar_without_subscription():
    lidar = DockingLidar.__new__(DockingLidar)
    lidar.node = FakeNode()
    lidar.last_scan = None
    lidar.last_received_at = 0.0
    lidar.last_valid_received_at = 0.0
    lidar.sequence = 0
    lidar.last_stamp_nanoseconds = 0
    lidar.last_error = 'no scan received'
    lidar._stream_valid = False
    lidar._transform = PlanarTransform(-0.166, 0.0, 3.1415)
    lidar._transform_z = 0.223
    return lidar


def scan_with_stamp(sec, nanosec):
    scan = LaserScan()
    scan.header.stamp.sec = sec
    scan.header.stamp.nanosec = nanosec
    return scan


def valid_scan_with_stamp(sec, nanosec):
    scan = scan_with_stamp(sec, nanosec)
    scan.header.frame_id = 'base_scan2'
    scan.angle_min = -1.0
    scan.angle_increment = 0.1
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [1.0]
    return scan


def test_duplicate_stamp_does_not_advance_or_refresh_scan():
    lidar = make_lidar_without_subscription()
    scan = scan_with_stamp(9, 900_000_000)

    lidar._scan_callback(scan)
    first_received_at = lidar.last_received_at
    assert lidar.sequence == 1
    assert lidar._stream_valid

    lidar._scan_callback(scan)
    assert lidar.sequence == 1
    assert lidar.last_received_at == first_received_at
    assert not lidar._stream_valid


def test_newer_stamp_recovers_after_duplicate():
    lidar = make_lidar_without_subscription()
    lidar._scan_callback(scan_with_stamp(9, 900_000_000))
    lidar._scan_callback(scan_with_stamp(9, 900_000_000))
    lidar._scan_callback(scan_with_stamp(9, 950_000_000))

    assert lidar.sequence == 2
    assert lidar._stream_valid


def test_invalid_new_messages_do_not_refresh_last_usable_scan():
    lidar = make_lidar_without_subscription()
    lidar._scan_callback(valid_scan_with_stamp(9, 900_000_000))

    assert lidar.snapshot() is not None
    last_valid_received_at = lidar.last_valid_received_at

    wrong_frame = valid_scan_with_stamp(9, 950_000_000)
    wrong_frame.header.frame_id = 'wrong_scan_frame'
    lidar._scan_callback(wrong_frame)

    assert lidar.snapshot() is None
    assert lidar.last_valid_received_at == last_valid_received_at
