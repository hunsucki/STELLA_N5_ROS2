"""Wall-clock control cadence and bounded recovery from stationary yaw stalls."""

import math
import time
from dataclasses import dataclass

import rclpy


def front_axle_goal_errors(longitudinal, lateral, heading, axle_x):
    """Aim the driven axle so a final axle-centered turn places base on goal.

    Errors and heading are expressed in the station frame. With an axle in
    front of base, a stationary turn moves base sideways; the target axle is
    therefore the target base plus axle_x along the station normal.
    """
    return (longitudinal + axle_x*(1-math.cos(heading)),
            lateral + axle_x*math.sin(heading))




def straight_entry_heading_budget(observations, distance, center_limit,
                                  half_width, side_margin, heading_limit,
                                  rear_reference_x, known_width=0.0,
                                  known_width_uncertainty=.01):
    """Bound a final straight corridor using five actual rail/panel samples.

    A nonzero known_width is an explicitly measured, fixed station profile;
    otherwise use the observed rail width. Reserve 3 mm for lateral error
    and include the body's front and rear corners, not just its center point.
    """
    if len(observations) != 5:
        return None
    values = [distance, center_limit, half_width, side_margin, heading_limit,
              rear_reference_x, known_width, known_width_uncertainty,
              *(v for row in observations for v in row)]
    if (not all(math.isfinite(v) for v in values) or distance <= 0
            or min(center_limit, half_width, side_margin, heading_limit) <= 0
            or abs(rear_reference_x + .2295) > .001
            or abs(half_width - .22) > .001
            or (known_width != 0.0 and not .50 <= known_width <= .58)
            or not .005 <= known_width_uncertainty <= .02):
        return None
    centers, widths, headings = zip(*observations)
    if known_width and (min(widths) < .50 or max(widths) > .58):
        return None
    if (max(centers)-min(centers) > (.015 if known_width else .0075)
            or max(widths)-min(widths) > .02
            or max(headings)-min(headings) > math.radians(.75)):
        return None
    lateral = max(abs(c) for c in centers) + .003
    center_room = center_limit - lateral
    # Aluminum reflections produced a single narrow fit amid four agreeing
    # scans in trial 42. Use the second narrowest of five (four supporting
    # measurements), subtract a further 3 mm, and retain the spread bound.
    supported_width = (known_width-known_width_uncertainty if known_width
                       else sorted(widths)[1]-.003)
    side_room = supported_width/2 - half_width - side_margin - lateral
    # At a measured station, the full remaining corridor is the acceptance
    # criterion. The steering deadband is not a second physical clearance
    # limit: a 16 mm residual can still fit safely in a 54 cm guide.
    if side_room <= 0 or (not known_width and center_room <= 0):
        return None
    sine_budget = min(1., side_room/(distance+.459))
    if not known_width:
        sine_budget = min(sine_budget, center_room/distance)
    budget = min(heading_limit, math.asin(sine_budget))
    # Leave measurement/settling headroom when committing to zero steering.
    if max(abs(a) for a in headings) + math.radians(.1) > budget:
        return None
    return budget


def remaining_straight_heading_budget(initial_budget, initial_distance,
                                      remaining_distance, lateral_spent,
                                      heading_limit):
    """Spend the original corridor clearance on travel and remaining body sweep.

    Only for an explicitly measured station width. Absolute lateral travel
    cannot be reclaimed by steering back; reserve the entire body length for
    the current heading even after most of the straight path is complete.
    """
    if (not all(math.isfinite(v) for v in (
            initial_budget, initial_distance, remaining_distance,
            lateral_spent, heading_limit))
            or not 0 < initial_budget <= heading_limit < math.pi/2
            or initial_distance <= 0 or remaining_distance < 0
            or lateral_spent < 0):
        return 0.0
    room = math.sin(initial_budget)*(initial_distance+.459)-lateral_spent
    return min(heading_limit, math.asin(min(
        1., max(0., room)/(remaining_distance+.459))))


@dataclass(frozen=True)
class StraightEntryPath:
    """Signed rectangle sweep in the measured guide frame, in metres/radians.

    The old symmetric bound adds lateral offset and yaw even when reversing
    moves the body toward the center. Keep their signs and check both front
    and rear corners, now and at the end of the remaining straight path.
    """
    observations: tuple
    anchor_heading: float
    supported_width: float
    half_width: float
    side_margin: float
    heading_limit: float

    def clearance(self, heading, progress, remaining, deviation=0.):
        if (not all(math.isfinite(v) for v in (heading, progress, remaining, deviation))
                or progress < 0 or remaining < 0 or deviation < 0
                or abs(heading) > self.heading_limit):
            return -math.inf
        worst = 0.
        for center, _, observed_heading in self.observations:
            # Preserve the five scans' heading uncertainty and a further 0.1deg
            # reserve. Evaluate the full angular interval analytically below.
            low = heading + observed_heading-self.anchor_heading-math.radians(.1)
            high = low + math.radians(.2)
            for future in (0., remaining):
                for x in (-.2295, .2295):
                    a, b = future-x, self.half_width
                    offset = progress*math.sin(observed_heading)-center
                    angles = [low, high]
                    for sign in (-1, 1):
                        critical = math.atan2(a, sign*b)
                        angles.extend(t for k in (-1, 0, 1)
                                      if low <= (t := critical+k*math.pi) <= high)
                    for angle in angles:
                        for sign in (-1, 1):
                            worst = max(worst, abs(
                                offset+a*math.sin(angle)+sign*b*math.cos(angle)))
        return (self.supported_width/2-worst-.003-deviation
                - progress*math.sin(math.radians(.1)))


def straight_entry_path(observations, distance, half_width, side_margin,
                        heading_limit, rear_reference_x, known_width,
                        known_width_uncertainty, anchor_heading):
    """Verify five measured samples before allowing a signed straight sweep."""
    if len(observations) != 5:
        return None
    values = [distance, half_width, side_margin, heading_limit, rear_reference_x,
              known_width, known_width_uncertainty, anchor_heading,
              *(v for row in observations for v in row)]
    if (not all(math.isfinite(v) for v in values) or distance <= 0
            or abs(rear_reference_x+.2295) > .001
            or abs(half_width-.22) > .001
            or not .50 <= known_width <= .58
            or not .005 <= known_width_uncertainty <= .02
            or not 0 < side_margin < known_width/2-half_width
            or not 0 < heading_limit <= math.radians(3.000001)):
        return None
    centers, widths, headings = zip(*observations)
    if (min(widths) < .50 or max(widths) > .58
            or max(widths)-min(widths) > .02
            or max(centers)-min(centers) > .015
            or max(headings)-min(headings) > math.radians(.75)
            or max(abs(h) for h in headings) > heading_limit):
        return None
    path = StraightEntryPath(tuple(observations), anchor_heading,
                             known_width-known_width_uncertainty,
                             half_width, side_margin, heading_limit)
    # Leave 2mm beyond the runtime stop threshold for the zero-steering handoff.
    return path if path.clearance(anchor_heading, 0., distance) >= side_margin+.002 else None


def spin_for(node, duration, should_stop=lambda: False):
    """Service callbacks for a full period, even when the executor is busy."""
    deadline = time.monotonic() + max(duration, 0.0)
    while rclpy.ok() and not should_stop():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        rclpy.spin_once(node, timeout_sec=min(remaining, 0.05))




class BackupStationaryYawController:
    """Bounded in-place correction, separate from gentle reverse steering.

    Uses the already validated rear plane for direction and independent gyro
    and wheel travel to stop when only the wheels respond. Never escalates
    beyond the speed used for the preceding stationary LiDAR alignment.
    """

    def __init__(self, max_speed=0.06, min_speed=0.025):
        if not (math.isfinite(max_speed) and math.isfinite(min_speed)
                and 0 < min_speed <= max_speed <= 0.06):
            raise ValueError('Stationary backup yaw requires 0 < min <= max <= 0.06')
        self.max_speed, self.min_speed = max_speed, min_speed
        self.started = None
        self.fault = None

    def update(self, error, gyro_yaw, wheel_yaw, now, tolerance):
        if self.fault:
            return 0.0
        if not all(math.isfinite(v) for v in (
                error, gyro_yaw, wheel_yaw, now, tolerance)):
            self.fault = 'non-finite stationary heading feedback'
            return 0.0
        if self.started is None:
            self.started = self.progress_at = now
            self.anchor_error = abs(error)
            self.previous_error = error
            self.previous_gyro, self.previous_wheel = gyro_yaw, wheel_yaw
            self.initial_gyro = gyro_yaw
            self.gyro_excursion = 0.0
            self.gyro_travel = self.wheel_travel = 0.0
        self.gyro_travel += abs(gyro_yaw - self.previous_gyro)
        self.gyro_excursion = max(self.gyro_excursion, abs(gyro_yaw - self.initial_gyro))
        delta = wheel_yaw - self.previous_wheel
        self.wheel_travel += abs(math.atan2(math.sin(delta), math.cos(delta)))
        self.previous_gyro, self.previous_wheel = gyro_yaw, wheel_yaw
        if (abs(error) <= tolerance
                or abs(self.previous_error) <= tolerance
                or error * self.previous_error < 0.0
                or abs(error) < self.anchor_error - math.radians(0.5)):
            self.anchor_error, self.progress_at = abs(error), now
        self.previous_error = error
        elapsed = now - self.started
        if elapsed >= 8.0:
            self.fault = 'stationary heading correction exceeded 8 seconds'
        elif max(self.wheel_travel, self.gyro_travel) >= math.radians(18):
            self.fault = 'stationary heading correction exceeded 18-degree travel budget'
        elif (elapsed >= 2.0 and self.wheel_travel >= math.radians(3)
              and self.gyro_excursion < math.radians(0.2)):
            self.fault = 'wheels rotate without independent gyro confirmation of chassis rotation'
        elif abs(error) > tolerance and now - self.progress_at >= 3.0:
            self.fault = 'rear plane heading did not converge within 3 seconds'
        if self.fault or abs(error) <= tolerance:
            return 0.0
        return math.copysign(
            max(self.min_speed, min(1.2 * abs(error), self.max_speed)), error)
