import math

import pytest

from teleop_joy.teleop_joy_node import button_pressed
from teleop_joy.teleop_joy_node import calculate_velocity
from teleop_joy.teleop_joy_node import clamp_axis
from teleop_joy.teleop_joy_node import trigger_value


def test_clamp_axis_limits_and_rejects_non_finite_values():
    assert clamp_axis(1.5) == 1.0
    assert clamp_axis(-2.0) == -1.0
    assert clamp_axis(math.nan) == 0.0


def test_trigger_value_converts_sdl_trigger_range():
    assert trigger_value(0.0) == 0.0
    assert trigger_value(-0.5) == 0.5
    assert trigger_value(-1.0) == 1.0
    assert trigger_value(0.2) == 0.0


def test_calculate_velocity_uses_rt_for_forward_and_lt_for_reverse():
    linear_x, angular_z = calculate_velocity(
        axes=[0.5, 0.0, 0.0, 0.0, -0.25, -1.0],
        forward_axis=5,
        reverse_axis=4,
        steering_axis=0,
        max_linear_speed=0.8,
        max_angular_speed=1.2,
    )

    assert linear_x == pytest.approx(0.6)
    assert angular_z == pytest.approx(0.6)


def test_steering_is_available_at_full_forward_speed():
    linear_x, angular_z = calculate_velocity(
        axes=[-1.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        forward_axis=5,
        reverse_axis=4,
        steering_axis=0,
        max_linear_speed=0.7,
        max_angular_speed=1.8,
    )

    assert linear_x == pytest.approx(0.7)
    assert angular_z == pytest.approx(-1.8)


def test_calculate_velocity_rejects_missing_axis():
    with pytest.raises(IndexError):
        calculate_velocity([0.0], 5, 4, 0, 0.7, 1.8)


def test_button_pressed_handles_disabled_or_missing_mapping():
    assert button_pressed([0, 1], 1)
    assert not button_pressed([0, 1], 0)
    assert not button_pressed([0, 1], -1)
    assert not button_pressed([0, 1], 5)
