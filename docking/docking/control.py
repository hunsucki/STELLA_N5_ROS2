"""Wall-clock control cadence and bounded recovery from stationary yaw stalls."""

import math
import time

import rclpy


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
    """

    def __init__(self, normal_limit=0.08, recovery_limit=0.15):
        if not (math.isfinite(normal_limit) and math.isfinite(recovery_limit)
                and 0.0 < normal_limit <= recovery_limit <= 0.15):
            raise ValueError('Precision yaw limits must satisfy 0 < normal <= recovery <= 0.15')
        self.normal_limit = normal_limit
        self.recovery_limit = recovery_limit
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
            max(min(abs(kp * error), self.normal_limit), self.recovery_floor), error)
        # Bound acceleration, including direction changes near the goal.
        dt = 0.05 if self.previous_time is None else min(max(now - self.previous_time, 0.0), 0.1)
        self.command += max(min(requested - self.command, 0.3 * dt), -0.3 * dt)
        self.previous_time = now
        return self.command
