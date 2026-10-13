"""User-confirmed pad contact is a nonzero distance from the LiDAR panel."""
import json
from pathlib import Path

import yaml
import pytest

from docking.lidar_geometry import has_consecutive_clearance_cluster
from test_backup_entry_completion import simulation

ROOT = Path(__file__).parents[1]


def test_measured_contact_was_outside_old_goal_and_has_stable_new_support():
    rows = json.loads((ROOT/'test/fixtures/contact_user79.json').read_text())['rows']
    profile = yaml.safe_load((ROOT/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters']
    target, tolerance = (profile[k] for k in ('backup_target_rear_clearance', 'backup_clearance_tolerance'))
    counts = []
    for goal, tol in [(.0145, .010), (target, tolerance)]:
        flags = [min(v for _, v in row) >= goal-tol and has_consecutive_clearance_cluster(
            row, goal+tol, 5, minimum=goal-tol, minimum_beam_span=6) for row in rows]
        run = best = 0
        for valid in flags:
            run = run+1 if valid else 0
            best = max(best, run)
        counts.append((sum(flags), best))
    assert counts == [(0, 0), (77, 31)]
    # A single isolated close return must not turn a far panel into completion.
    assert not has_consecutive_clearance_cluster(
        [(0, target), *[(i, .05) for i in range(1, 12)]],
        target+tolerance, 5, minimum=target-tolerance, minimum_beam_span=6)


def test_current_contact_completes_while_stopped_without_more_reverse(simulation):
    motion, state = simulation
    profile = yaml.safe_load((ROOT/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters']
    motion.node.parameters.update(profile)
    state.gap = .028
    rows = json.loads((ROOT/'test/fixtures/contact_user79.json').read_text())['rows']
    motion._rear_clearances = lambda snapshot: rows[min(snapshot.sequence, len(rows)-1)]
    assert motion.backup_with_lidar(), motion.node.get_logger().error.call_args
    assert all(v == w == 0 for _, _, _, v, w in state.sent)


def test_second_contact_has_no_supported_cluster_with_three_mm_tolerance():
    rows = json.loads((ROOT/'test/fixtures/contact_user80.json').read_text())['rows']
    for row in rows:
        assert not has_consecutive_clearance_cluster(
            row['values'], .031, 5, minimum=.025, minimum_beam_span=row['beam_span'])
        assert has_consecutive_clearance_cluster(
            row['values'], .034, 5, minimum=.022, minimum_beam_span=row['beam_span'])
        assert min(v for _, v in row['safety_values']) >= .022


@pytest.mark.parametrize('old_tolerance,expected', [(True, False), (False, True)])
def test_second_contact_replays_blocked_deadlock_without_extra_motion(
        simulation, old_tolerance, expected):
    motion, state = simulation
    profile = yaml.safe_load((ROOT/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters']
    motion.node.parameters.update(profile)
    # Isolate the final distance decision; the real run had already verified
    # guides and committed to straight entry before reaching this position.
    motion.node.parameters['use_lidar_guide_centering'] = False
    if old_tolerance:
        motion.node.parameters['backup_clearance_tolerance'] = .003
    rows = json.loads((ROOT/'test/fixtures/contact_user80.json').read_text())['rows']
    state.gap = .032
    motion._rear_clearances = lambda s: rows[min(s.sequence, len(rows)-1)]['values']
    motion._rear_safety_clearances = lambda s: rows[min(s.sequence, len(rows)-1)]['safety_values']
    assert motion.backup_with_lidar() is expected
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
    if not expected:
        assert 'remains blocked' in motion.node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('fault', ['far_panel', 'single_return', 'unsafe_side', 'overrun'])
def test_contact_tolerance_does_not_accept_unsupported_or_unsafe_contact(simulation, fault):
    motion, state = simulation
    profile = yaml.safe_load((ROOT/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters']
    motion.node.parameters.update(profile)
    motion._rear_safety_clearances = lambda _: [(0, .027)]
    if fault == 'unsafe_side':
        motion._rear_clearances = lambda _: [(i, .032) for i in range(12)]
        motion._rear_safety_clearances = lambda _: [(0, .021)]
    elif fault == 'overrun':
        motion._rear_clearances = lambda _: [(i, .020) for i in range(12)]
    elif fault == 'single_return':
        motion._rear_clearances = lambda _: [(0, .028), *[(i, .050) for i in range(1, 12)]]
    else:
        motion._rear_clearances = lambda _: [(i, .050) for i in range(12)]
    assert not motion.backup_with_lidar()
    assert state.linear == state.angular == 0
    assert all(v == w == 0 for _, _, _, v, w in state.sent)
