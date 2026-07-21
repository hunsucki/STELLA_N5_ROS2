# Copyright 2026 NTRex Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for hourly capture directory allocation."""

from datetime import datetime, timezone

from gimbal_camera_capture.storage import HourlyRunDirectory


def test_same_hour_uses_same_directory(tmp_path):
    """Two captures in one hour share a run directory."""
    allocator = HourlyRunDirectory(tmp_path)

    first = allocator.for_time(datetime(2026, 7, 21, 10, 1, tzinfo=timezone.utc))
    second = allocator.for_time(
        datetime(2026, 7, 21, 10, 59, tzinfo=timezone.utc)
    )

    assert first == second
    assert first.name == 'run_20260721_1'


def test_new_hour_increments_run_number(tmp_path):
    """A later capture hour gets the next run number."""
    allocator = HourlyRunDirectory(tmp_path)

    first = allocator.for_time(datetime(2026, 7, 21, 10, 0, tzinfo=timezone.utc))
    second = allocator.for_time(datetime(2026, 7, 21, 11, 0, tzinfo=timezone.utc))

    assert first.name == 'run_20260721_1'
    assert second.name == 'run_20260721_2'


def test_new_date_restarts_run_number(tmp_path):
    """Run numbering starts from one again on a new date."""
    allocator = HourlyRunDirectory(tmp_path)

    allocator.for_time(datetime(2026, 7, 21, 23, 0, tzinfo=timezone.utc))
    next_day = allocator.for_time(
        datetime(2026, 7, 22, 0, 0, tzinfo=timezone.utc)
    )

    assert next_day.name == 'run_20260722_1'
