"""Tests for durable date/run/camera capture storage."""

from datetime import datetime, timedelta, timezone

from gimbal_camera_capture.storage import DatedCapturePaths
import pytest
import yaml


STARTED_AT = datetime(2026, 9, 3, 15, 36, 33, tzinfo=timezone.utc)
MISSION_ID = 'mission-123'
MISSION_CONTEXT = {
    'mission_id': MISSION_ID,
    'map_id': 'greenhouse-map',
    'map_frame': 'map',
    'zone_revision': 'zones-v1',
    'zones': {
        name: {
            'min_x': float(index),
            'min_y': 0.0,
            'max_x': float(index + 1),
            'max_y': 1.0,
        }
        for index, name in enumerate(('A', 'B', 'C', 'D', 'E'))
    },
}


def load_metadata(run_directory):
    """Load a run's YAML metadata for assertions."""
    with (run_directory / 'metadata.yaml').open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def start_mission(allocator):
    """Start the standard test mission and return its directory."""
    run_directory, created, status = allocator.start_mission_run(
        STARTED_AT,
        MISSION_CONTEXT,
    )
    assert created is True
    assert status == 'running'
    return run_directory


def test_first_legacy_capture_creates_schema_v2_run(tmp_path):
    """The legacy capture path also uses the new run structure."""
    allocator = DatedCapturePaths(tmp_path)

    directory, paths = allocator.allocate(STARTED_AT)

    assert directory == tmp_path / '20260903' / 'run_1'
    assert paths['left'] == directory / 'left' / '153633_1.jpg'
    assert paths['right'] == directory / 'right' / '153633_1.jpg'
    metadata = load_metadata(directory)
    assert metadata['schema_version'] == 2
    assert metadata['status'] == 'running'
    assert metadata['mission_id'] == ''


def test_mission_start_persists_map_and_zone_snapshot(tmp_path):
    """A service run stores its complete mission snapshot."""
    allocator = DatedCapturePaths(tmp_path)

    run_directory = start_mission(allocator)

    metadata = load_metadata(run_directory)
    assert metadata['mission_id'] == MISSION_ID
    assert metadata['map'] == {
        'id': 'greenhouse-map',
        'frame_id': 'map',
    }
    assert metadata['zone_config']['revision'] == 'zones-v1'
    assert set(metadata['zone_config']['zones']) == set('ABCDE')


def test_same_mission_start_is_idempotent_even_after_restart(tmp_path):
    """A repeated mission start reuses the durable running run."""
    first_allocator = DatedCapturePaths(tmp_path)
    first_run = start_mission(first_allocator)
    restarted_allocator = DatedCapturePaths(tmp_path)

    run_directory, created, status = (
        restarted_allocator.start_mission_run(
            STARTED_AT + timedelta(seconds=5),
            MISSION_CONTEXT,
        )
    )

    assert run_directory == first_run
    assert created is False
    assert status == 'running'
    assert restarted_allocator.active_run_directory == first_run


def test_other_mission_is_rejected_while_run_is_active(tmp_path):
    """Two missions cannot write into one active storage manager."""
    allocator = DatedCapturePaths(tmp_path)
    start_mission(allocator)
    other_context = dict(MISSION_CONTEXT, mission_id='mission-456')

    with pytest.raises(RuntimeError, match='another mission run'):
        allocator.start_mission_run(STARTED_AT, other_context)


def test_same_second_increments_sequence_inside_active_run(tmp_path):
    """Repeated requests in one second never overwrite a pair."""
    allocator = DatedCapturePaths(tmp_path)
    allocator.allocate(STARTED_AT)

    _, second_paths = allocator.allocate(STARTED_AT)

    assert second_paths['left'].name == '153633_2.jpg'
    assert second_paths['right'].name == '153633_2.jpg'


def test_existing_file_on_either_side_skips_pair(tmp_path):
    """An existing image on one side advances both filenames."""
    allocator = DatedCapturePaths(tmp_path)
    run_directory = allocator.start_run(STARTED_AT)
    (run_directory / 'left' / '153633_1.jpg').touch()

    _, paths = allocator.allocate(STARTED_AT)

    assert paths['left'].name == '153633_2.jpg'
    assert paths['right'].name == '153633_2.jpg'


def test_service_capture_records_context_and_relative_paths(tmp_path):
    """Capture metadata connects request, zone, pose, and paired files."""
    allocator = DatedCapturePaths(tmp_path)
    run_directory = start_mission(allocator)
    _, paths = allocator.allocate(STARTED_AT)
    for path in paths.values():
        path.touch()
    robot_pose = {
        'frame_id': 'map',
        'stamp': '2026-09-03T15:36:32.900000000+00:00',
        'source': 'amcl',
        'x': 1.25,
        'y': -2.5,
        'yaw': 0.75,
        'covariance': [0.0] * 36,
    }

    _, capture = allocator.record_service_capture(
        captured_at=STARTED_AT,
        files={name: str(path) for name, path in paths.items()},
        errors={},
        success=True,
        request_id='capture-000001',
        zone_id='A',
        requested_at='2026-09-03T15:36:32.800000000+00:00',
        robot_pose=robot_pose,
    )

    assert capture['capture_id'] == 1
    assert capture['request_id'] == 'capture-000001'
    assert capture['zone_id'] == 'A'
    assert capture['robot_pose'] == robot_pose
    assert capture['files'] == {
        'left': 'left/153633_1.jpg',
        'right': 'right/153633_1.jpg',
    }
    metadata = load_metadata(run_directory)
    assert metadata['summary']['successful_pairs'] == 1
    assert metadata['summary']['image_files'] == 2


def test_request_id_retry_returns_record_without_duplicate(tmp_path):
    """A repeated request id cannot append a second capture entry."""
    allocator = DatedCapturePaths(tmp_path)
    run_directory = start_mission(allocator)

    _, first = allocator.record_service_capture(
        STARTED_AT,
        {},
        {'left': 'offline', 'right': 'offline'},
        False,
        request_id='capture-000001',
        zone_id='A',
    )
    _, retry = allocator.record_service_capture(
        STARTED_AT + timedelta(seconds=1),
        {},
        {},
        True,
        request_id='capture-000001',
        zone_id='A',
    )

    assert retry == first
    metadata = load_metadata(run_directory)
    assert len(metadata['captures']) == 1
    assert metadata['summary']['capture_requests'] == 1


def test_finish_is_idempotent_and_marks_run_ready(tmp_path):
    """Finish commits metadata before a durable READY marker."""
    allocator = DatedCapturePaths(tmp_path)
    run_directory = start_mission(allocator)
    finished_at = STARTED_AT + timedelta(minutes=30)

    directory, metadata_path, idempotent = (
        allocator.finish_mission_run(
            finished_at,
            run_directory.name,
            MISSION_ID,
        )
    )

    assert directory == run_directory
    assert metadata_path == run_directory / 'metadata.yaml'
    assert idempotent is False
    assert (run_directory / 'READY').is_file()
    assert load_metadata(run_directory)['status'] == 'completed'

    restarted_allocator = DatedCapturePaths(tmp_path)
    retry = restarted_allocator.finish_mission_run(
        finished_at,
        run_directory.name,
        MISSION_ID,
    )
    assert retry == (run_directory, metadata_path, True)


def test_abort_is_idempotent_and_never_leaves_ready(tmp_path):
    """Abort preserves images and metadata but removes READY."""
    allocator = DatedCapturePaths(tmp_path)
    run_directory = start_mission(allocator)

    _, metadata_path, idempotent = allocator.abort_mission_run(
        STARTED_AT + timedelta(minutes=1),
        run_directory.name,
        MISSION_ID,
        'ESTOP',
    )

    assert idempotent is False
    assert not (run_directory / 'READY').exists()
    metadata = load_metadata(run_directory)
    assert metadata['status'] == 'aborted'
    assert metadata['abort_reason'] == 'ESTOP'

    retry = DatedCapturePaths(tmp_path).abort_mission_run(
        STARTED_AT + timedelta(minutes=2),
        run_directory.name,
        MISSION_ID,
        'duplicate',
    )
    assert retry == (run_directory, metadata_path, True)


def test_run_and_mission_mismatch_are_rejected(tmp_path):
    """Finish cannot commit a run using mismatched identifiers."""
    allocator = DatedCapturePaths(tmp_path)
    run_directory = start_mission(allocator)

    with pytest.raises(RuntimeError, match='do not match'):
        allocator.finish_mission_run(
            STARTED_AT,
            run_directory.name,
            'wrong-mission',
        )


def test_new_runs_follow_highest_number_for_each_date(tmp_path):
    """Run numbering is monotonic and independent for each date."""
    date_directory = tmp_path / '20260903'
    (date_directory / 'run_1').mkdir(parents=True)
    (date_directory / 'run_3').mkdir()
    allocator = DatedCapturePaths(tmp_path)

    run_directory = allocator.start_run(STARTED_AT)
    allocator.finish_run(STARTED_AT + timedelta(hours=1))
    next_date = STARTED_AT + timedelta(days=1)
    next_run = allocator.start_run(next_date)

    assert run_directory.name == 'run_4'
    assert next_run == tmp_path / '20260904' / 'run_1'
