"""Pure 2-D geometry helpers for the docking LiDAR."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class PlanarTransform:
    """Pose of the LiDAR frame expressed in the robot base frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ProjectedPoint:
    """One LaserScan return projected into the robot base frame."""

    beam_index: int
    x: float
    y: float
    distance: float


class UniqueScanStability:
    """Count stable observations only when a new scan sequence is seen."""

    def __init__(self, required: int, initial_sequence: int = -1) -> None:
        self.required = max(int(required), 1)
        self.last_sequence = initial_sequence
        self.count = 0

    def observe(self, sequence: int, stable: bool) -> bool:
        if sequence == self.last_sequence:
            return self.count >= self.required

        self.last_sequence = sequence
        if stable:
            self.count += 1
        else:
            self.count = 0
        return self.count >= self.required

    def reset(self) -> None:
        self.count = 0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def line_orientation_error(line_angle: float, target_angle: float) -> float:
    """Shortest line-orientation error, treating angles pi apart as equal."""
    delta = normalize_angle(line_angle - target_angle)
    return 0.5 * math.atan2(math.sin(2.0 * delta), math.cos(2.0 * delta))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def scan_metadata_is_valid(
        ranges: Sequence[float], angle_min: float, angle_increment: float,
        range_min: float, range_max: float) -> bool:
    return (
        bool(ranges)
        and math.isfinite(angle_min)
        and math.isfinite(angle_increment)
        and abs(angle_increment) > 1e-12
        and math.isfinite(range_min)
        and math.isfinite(range_max)
        and range_min >= 0.0
        and range_max > range_min
    )


def effective_range_limits(
        scan_min: float, scan_max: float,
        configured_min: float, configured_max: float) -> tuple[float, float] | None:
    if not all(math.isfinite(value) for value in (
            scan_min, scan_max, configured_min, configured_max)):
        return None
    lower = max(scan_min, configured_min, 0.0)
    upper = min(scan_max, configured_max)
    if not math.isfinite(lower) or not math.isfinite(upper) or upper <= lower:
        return None
    return lower, upper


def project_scan_to_base(
        ranges: Sequence[float], angle_min: float, angle_increment: float,
        scan_range_min: float, scan_range_max: float,
        transform: PlanarTransform,
        sector_center: float, sector_width: float,
        configured_min_range: float,
        configured_max_range: float) -> list[ProjectedPoint] | None:
    """Project valid returns whose ray direction lies in a base-frame sector."""
    if not scan_metadata_is_valid(
            ranges, angle_min, angle_increment,
            scan_range_min, scan_range_max):
        return None

    limits = effective_range_limits(
        scan_range_min, scan_range_max,
        configured_min_range, configured_max_range)
    if limits is None:
        return None

    lower, upper = limits
    half_width = abs(sector_width) / 2.0
    if not math.isfinite(sector_center) or not math.isfinite(half_width):
        return None
    if half_width <= 0.0:
        return []

    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    points: list[ProjectedPoint] = []

    for beam_index, distance in enumerate(ranges):
        angle = angle_min + beam_index * angle_increment
        base_ray_angle = normalize_angle(angle + transform.yaw)
        if abs(normalize_angle(base_ray_angle - sector_center)) > half_width:
            continue
        if not math.isfinite(distance) or not lower <= distance <= upper:
            continue

        scan_x = distance * math.cos(angle)
        scan_y = distance * math.sin(angle)
        points.append(ProjectedPoint(
            beam_index=beam_index,
            x=transform.x + cos_yaw * scan_x - sin_yaw * scan_y,
            y=transform.y + sin_yaw * scan_x + cos_yaw * scan_y,
            distance=distance,
        ))

    return points


def rear_clearances(
        points: Sequence[ProjectedPoint], rear_reference_x: float
        ) -> list[tuple[int, float]]:
    """Return beam index and longitudinal gap from the rear body reference."""
    return [
        (point.beam_index, rear_reference_x - point.x)
        for point in points
    ]


def has_consecutive_clearance_cluster(
        indexed_clearances: Sequence[tuple[int, float]],
        threshold: float, required_points: int,
        max_beam_gap: int = 2,
        minimum: float = -math.inf,
        minimum_beam_span: int = 0) -> bool:
    """Require several neighboring beams before declaring docking complete."""
    required = max(int(required_points), 1)
    run = 0
    first_index: int | None = None
    previous_index: int | None = None

    for beam_index, clearance in indexed_clearances:
        if minimum <= clearance <= threshold:
            if (
                    previous_index is not None
                    and beam_index - previous_index <= max(max_beam_gap, 1)):
                run += 1
            else:
                run = 1
                first_index = beam_index
            previous_index = beam_index
            if (
                    run >= required
                    and first_index is not None
                    and beam_index - first_index >= max(minimum_beam_span, 0)):
                return True
        else:
            run = 0
            first_index = None
            previous_index = None

    return False


def required_rear_range_at_completion(
        sensor_x: float, rear_reference_x: float,
        target_clearance: float, tolerance: float) -> float:
    """Maximum rear-facing range that can satisfy the completion condition."""
    completion_plane_x = rear_reference_x - target_clearance - tolerance
    return sensor_x - completion_plane_x


def scan_is_fresh(received_at: float, now: float, max_age: float) -> bool:
    return received_at > 0.0 and 0.0 <= now - received_at <= max_age


def header_stamp_is_acceptable(
        stamp_nanoseconds: int, previous_stamp_nanoseconds: int,
        now_nanoseconds: int, max_age_nanoseconds: int,
        future_tolerance_nanoseconds: int) -> bool:
    if stamp_nanoseconds <= 0:
        return False
    if stamp_nanoseconds <= previous_stamp_nanoseconds:
        return False
    age = now_nanoseconds - stamp_nanoseconds
    return -future_tolerance_nanoseconds <= age <= max_age_nanoseconds
