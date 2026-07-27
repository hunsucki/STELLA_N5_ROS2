"""Tests for date/camera capture path allocation."""

from datetime import datetime, timezone

from gimbal_camera_capture.storage import DatedCapturePaths


def test_allocates_matching_left_and_right_paths(tmp_path):
    """Both cameras get the requested date, time, and sequence."""
    allocator = DatedCapturePaths(tmp_path)

    directory, paths = allocator.allocate(
        datetime(2026, 7, 26, 15, 36, 33, tzinfo=timezone.utc)
    )

    assert directory == tmp_path / '20260726'
    assert paths['left'] == directory / 'left' / '153633_1.jpg'
    assert paths['right'] == directory / 'right' / '153633_1.jpg'
    assert paths['left'].parent.is_dir()
    assert paths['right'].parent.is_dir()


def test_same_second_increments_sequence(tmp_path):
    """Repeated requests in one second never overwrite the first pair."""
    allocator = DatedCapturePaths(tmp_path)
    timestamp = datetime(2026, 7, 26, 15, 36, 33, tzinfo=timezone.utc)

    allocator.allocate(timestamp)
    _, second_paths = allocator.allocate(timestamp)

    assert second_paths['left'].name == '153633_2.jpg'
    assert second_paths['right'].name == '153633_2.jpg'


def test_existing_file_in_either_camera_directory_skips_pair(tmp_path):
    """An existing image on either side advances both sequence numbers."""
    allocator = DatedCapturePaths(tmp_path)
    timestamp = datetime(2026, 7, 26, 15, 36, 33, tzinfo=timezone.utc)
    existing = tmp_path / '20260726' / 'left' / '153633_1.jpg'
    existing.parent.mkdir(parents=True)
    existing.touch()

    _, paths = allocator.allocate(timestamp)

    assert paths['left'].name == '153633_2.jpg'
    assert paths['right'].name == '153633_2.jpg'


def test_new_date_uses_a_new_date_directory(tmp_path):
    """A new calendar date gets independent left/right directories."""
    allocator = DatedCapturePaths(tmp_path)

    directory, paths = allocator.allocate(
        datetime(2026, 7, 27, 0, 0, 1, tzinfo=timezone.utc)
    )

    assert directory.name == '20260727'
    assert paths['left'].name == '000001_1.jpg'
    assert paths['right'].name == '000001_1.jpg'
