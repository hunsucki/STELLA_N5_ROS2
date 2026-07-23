import math

from docking.dock_turn_backup import DockTurnBackup
from docking.docking_lidar import DockingLidar
from docking.lidar_alignment import LidarPlaneAligner
from docking.motion import MotionController


class ParameterCapture:
    def __init__(self):
        self.defaults = {}

    def declare_parameter(self, name, default):
        self.defaults[name] = default


def test_declared_lidar_defaults_match_new_mount_contract():
    node = ParameterCapture()
    MotionController.declare_parameters(node)
    LidarPlaneAligner.declare_parameters(node)
    DockingLidar.declare_parameters(node)
    DockTurnBackup._declare_safety_parameters(node)

    assert node.defaults['docking_lidar_topic'] == '/scan_2'
    assert node.defaults['docking_lidar_frame'] == 'base_scan2'
    assert node.defaults['docking_lidar_scan_max_age_sec'] == 0.30
    assert node.defaults['backup_lidar_sector_center_base'] == math.pi
    assert math.isnan(node.defaults['backup_lidar_sector_center'])
    assert node.defaults['backup_lidar_min_range'] == 0.05
    assert node.defaults['backup_rear_reference_x'] == -0.2295
    assert node.defaults['backup_lidar_success_min_points'] >= 5
    assert node.defaults['backup_lidar_success_min_angle_span'] >= math.radians(3.0)
    assert node.defaults['backup_lidar_stable_cycles'] >= 3
    assert node.defaults['lidar_align_sector_center_base'] == math.pi
    assert math.isnan(node.defaults['lidar_align_sector_center'])
    assert node.defaults['development_test_mode'] is True
