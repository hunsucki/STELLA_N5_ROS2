"""Charging success requires uninterrupted contact and finite battery current."""

from types import SimpleNamespace
from unittest.mock import Mock

from docking.charging import ChargingVerifier
import pytest
from sensor_msgs.msg import BatteryState


@pytest.fixture
def verifier(monkeypatch):
    parameters = {}
    node = SimpleNamespace(
        declare_parameter=lambda name, default: parameters.setdefault(name, default),
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        create_publisher=Mock(), create_subscription=Mock(),
    )
    ChargingVerifier.declare_parameters(node)
    clock = [100.0]
    monkeypatch.setattr('docking.charging.time.monotonic', lambda: clock[0])
    return ChargingVerifier(node, lambda: False), clock


def charging_sample(current=0.2, present=True):
    return BatteryState(
        present=present, current=current,
        power_supply_status=BatteryState.POWER_SUPPLY_STATUS_CHARGING)


def deliver(verifier):
    verifier._available_callback(charging_sample())
    verifier._battery_callback(charging_sample())


def test_continuous_charge_succeeds_only_after_stability_window(verifier):
    charge, clock = verifier
    for second in range(4):
        clock[0] = 100.0 + second
        deliver(charge)
        assert charge._charging_is_stable(clock[0]) == (second == 3)


@pytest.mark.parametrize('stream', ['battery', 'available', 'both'])
def test_stream_gap_restarts_stability_even_without_polling(verifier, stream):
    charge, clock = verifier
    deliver(charge)
    for second in range(1, 5):
        clock[0] = 100.0 + second
        if stream == 'battery':
            charge._available_callback(charging_sample())
        elif stream == 'available':
            charge._battery_callback(charging_sample())
    deliver(charge)
    assert charge.charging_since == clock[0]
    assert not charge._charging_is_stable(clock[0])
    for second in range(1, 4):
        clock[0] = 104.0 + second
        deliver(charge)
    assert charge._charging_is_stable(clock[0])


def test_contact_loss_restarts_window_between_polls(verifier):
    charge, clock = verifier
    deliver(charge)
    clock[0] += 2.0
    charge._available_callback(charging_sample(present=False))
    assert charge.charging_since is None
    deliver(charge)
    clock[0] += 1.0
    deliver(charge)
    assert not charge._charging_is_stable(clock[0])


@pytest.mark.parametrize('current', [float('inf'), float('-inf'), float('nan'), -0.2, 0.0])
def test_invalid_or_noncharging_current_cannot_confirm_charge(verifier, current):
    charge, clock = verifier
    deliver(charge)
    charge._battery_callback(charging_sample(current=current))
    assert not charge.already_charging()
    assert charge.charging_since is None
    assert not charge._charging_is_stable(clock[0])


def test_polling_expired_samples_clears_stability(verifier):
    charge, clock = verifier
    deliver(charge)
    clock[0] += 3.1
    assert not charge._charging_is_stable(clock[0])
    assert charge.charging_since is None
