"""Wall-clock control cadence and bounded recovery from stationary yaw stalls."""

import math
import time

import rclpy


def front_axle_goal_errors(longitudinal, lateral, heading, axle_x):
    """Aim the driven axle so a final axle-centered turn places base on goal.

    Errors and heading are expressed in the station frame. With an axle in
    front of base, a stationary turn moves base sideways; the target axle is
    therefore the target base plus axle_x along the station normal.
    """
    return (longitudinal + axle_x*(1-math.cos(heading)),
            lateral + axle_x*math.sin(heading))


class CenterlineLatch:
    """Confirm a front control stage without changing its success tolerance."""

    def __init__(self, tolerance, entering_tolerance=None):
        self.tolerance = tolerance
        self.entering_tolerance = (tolerance if entering_tolerance is None
                                   else entering_tolerance)
        self.centered = False
        self.pending = False
        self.last_sequence = None
        self.count = 0

    def reset(self):
        self.centered = self.pending = False
        self.last_sequence = None
        self.count = 0

    def update(self, sequence, lateral, heading_settled, settled_lateral=None):
        settled_lateral = lateral if settled_lateral is None else settled_lateral
        if not all(math.isfinite(v) for v in (lateral, settled_lateral)):
            self.reset()
            return False
        inside = abs(lateral) <= self.entering_tolerance
        settled_inside = abs(settled_lateral) <= self.tolerance
        if sequence != self.last_sequence:
            self.last_sequence = sequence
            if not self.centered:
                self.count = self.count+1 if inside else 0
                if self.count >= 3:
                    self.centered, self.count = True, 0
            else:
                # Finish yaw across small boundary noise. Once yaw settles,
                # the original tolerance must pass again before straight drive.
                outside = (abs(lateral) > self.tolerance+.01
                           or (heading_settled and not settled_inside))
                self.count = self.count+1 if outside else 0
                if self.count >= 3:
                    self.centered, self.count = False, 0
        self.pending = ((not self.centered and inside)
                        or (self.centered and heading_settled and not settled_inside))
        return self.centered


def straight_entry_heading_budget(observations, distance, center_limit,
                                  half_width, side_margin, heading_limit,
                                  rear_reference_x, known_width=0.0,
                                  known_width_uncertainty=.01):
    """Bound a final straight corridor using five actual rail/panel samples.

    A nonzero known_width is an explicitly measured, fixed station profile;
    otherwise use the observed rail width. Reserve 3 mm for lateral error
    and bound rotation of the whole 459 mm body, not just its center point.
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
    # A calibrated guide may accept small final residuals provided the whole
    # remaining path stays inside its physical side margin. The 15 mm center
    # gate still applies to all five observed entry positions.
    if known_width and max(abs(c) for c in centers) > center_limit:
        return None
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


def spin_for(node, duration, should_stop=lambda: False):
    """Service callbacks for a full period, even when the executor is busy."""
    deadline = time.monotonic() + max(duration, 0.0)
    while rclpy.ok() and not should_stop():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        rclpy.spin_once(node, timeout_sec=min(remaining, 0.05))


class PrecisionYawController:
    """Increase rotation command only when both motion sensors report no rotation.

    Never infer a stalled chassis from a frozen fused heading alone. Encoder
    or gyro motion with no heading progress requires stopping, not a stronger command.
    An optional nominal minimum avoids ineffective low-speed final commands;
    it does not bypass the no-progress guard or acceleration limit. The caller
    stops the command inside its heading tolerance.
    """

    def __init__(self, normal_limit=0.08, recovery_limit=0.15, minimum_speed=0.0):
        if not (math.isfinite(normal_limit) and math.isfinite(recovery_limit)
                and 0.0 < normal_limit <= recovery_limit <= 0.15):
            raise ValueError('Precision yaw limits must satisfy 0 < normal <= recovery <= 0.15')
        if not math.isfinite(minimum_speed) or not 0 <= minimum_speed <= normal_limit:
            raise ValueError('Precision yaw minimum must satisfy 0 <= minimum <= normal')
        self.normal_limit = normal_limit
        self.recovery_limit = recovery_limit
        self.minimum_speed = minimum_speed
        self.reset()

    def reset(self):
        self.anchor_error = None
        self.progress_at = None
        self.recovery_floor = 0.0
        self.command = 0.0
        self.previous_time = None
        self.fault = None

    def update(self, error, gyro_rate, wheel_rate, now, kp=1.0, allow_recovery=True):
        if self.fault:
            return 0.0
        if not all(math.isfinite(v) for v in (error, gyro_rate, wheel_rate, now, kp)):
            self.fault = 'non-finite yaw feedback'
            return 0.0
        if self.anchor_error is None:
            self.anchor_error, self.progress_at = error, now
        # Translation can coast briefly into the yaw phase. Measure recovery
        # from that peak error, without granting more time for wrong-way motion.
        if abs(error) > abs(self.anchor_error):
            self.anchor_error = error
        # Count actual error reduction, rather than small alternating jitter.
        if abs(error) < abs(self.anchor_error) - math.radians(0.5):
            self.anchor_error, self.progress_at = error, now
        stalled_for = now - self.progress_at
        stationary = max(abs(gyro_rate), abs(wheel_rate)) <= math.radians(0.5)
        if stalled_for >= 1.0:
            if not stationary:
                if stalled_for >= 2.0:
                    self.fault = 'heading is not converging despite measured rotation'
                    return 0.0
            else:
                if not allow_recovery:
                    if stalled_for >= 2.0:
                        self.fault = 'cannot recover yaw stall without fresh independent gyro'
                    return 0.0
                self.recovery_floor = min(
                    self.recovery_limit, self.normal_limit + 0.04 * (stalled_for - 1.0))
                if stalled_for >= 4.0:
                    self.fault = 'no yaw progress after bounded stationary recovery'
                    return 0.0
        requested = math.copysign(
            max(min(abs(kp * error), self.normal_limit),
                self.minimum_speed if error != 0.0 else 0.0,
                self.recovery_floor), error)
        # Bound acceleration, including direction changes near the goal.
        dt = 0.05 if self.previous_time is None else min(max(now - self.previous_time, 0.0), 0.1)
        self.command += max(min(requested - self.command, 0.3 * dt), -0.3 * dt)
        self.previous_time = now
        return self.command


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
