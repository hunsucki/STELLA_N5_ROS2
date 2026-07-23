"""Hourly run-directory allocation for captured images."""

from datetime import datetime
from pathlib import Path
import re


class HourlyRunDirectory:
    """Map each local clock hour to one numbered run directory."""

    _MARKER_NAME = '.capture_hour'

    def __init__(self, base_directory: Path) -> None:
        """Initialize the allocator with an expanded absolute directory."""
        self._base_directory = base_directory.expanduser().resolve()

    @property
    def base_directory(self) -> Path:
        """Return the capture base directory."""
        return self._base_directory

    def for_time(self, timestamp: datetime) -> Path:
        """Return the run directory shared by captures in the same hour."""
        date_key = timestamp.strftime('%Y%m%d')
        hour_key = timestamp.strftime('%Y%m%d%H')
        self._base_directory.mkdir(parents=True, exist_ok=True)

        run_pattern = re.compile(rf'^run_{date_key}_(\d+)$')
        indexed_directories = []
        for candidate in self._base_directory.iterdir():
            if not candidate.is_dir():
                continue
            match = run_pattern.match(candidate.name)
            if match is None:
                continue
            indexed_directories.append((int(match.group(1)), candidate))

        for _, candidate in sorted(indexed_directories):
            marker_path = candidate / self._MARKER_NAME
            try:
                if marker_path.read_text(encoding='utf-8').strip() == hour_key:
                    return candidate
            except FileNotFoundError:
                continue

        next_index = max((index for index, _ in indexed_directories), default=0) + 1
        run_directory = self._base_directory / f'run_{date_key}_{next_index}'
        run_directory.mkdir(parents=False, exist_ok=False)
        (run_directory / self._MARKER_NAME).write_text(
            hour_key + '\n',
            encoding='utf-8',
        )
        return run_directory
