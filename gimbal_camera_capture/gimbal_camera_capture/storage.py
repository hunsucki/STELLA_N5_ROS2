"""Allocate paired left/right paths for captured images."""

from datetime import datetime
from pathlib import Path
import threading
from typing import Dict, Tuple


class DatedCapturePaths:
    """Allocate matching filenames below date and camera directories."""

    def __init__(self, base_directory: Path) -> None:
        """Initialize the allocator with an expanded absolute directory."""
        self._base_directory = base_directory.expanduser().resolve()
        self._reserved_paths = set()
        self._lock = threading.Lock()

    @property
    def base_directory(self) -> Path:
        """Return the capture base directory."""
        return self._base_directory

    def allocate(self, timestamp: datetime) -> Tuple[Path, Dict[str, Path]]:
        """Create date/camera folders and reserve one paired filename."""
        date_directory = self._base_directory / timestamp.strftime('%Y%m%d')
        camera_directories = {
            'left': date_directory / 'left',
            'right': date_directory / 'right',
        }
        filename_prefix = timestamp.strftime('%H%M%S')

        with self._lock:
            for directory in camera_directories.values():
                directory.mkdir(parents=True, exist_ok=True)

            sequence = 1
            while True:
                filename = f'{filename_prefix}_{sequence}.jpg'
                paths = {
                    name: directory / filename
                    for name, directory in camera_directories.items()
                }
                if all(
                    not path.exists() and path not in self._reserved_paths
                    for path in paths.values()
                ):
                    self._reserved_paths.update(paths.values())
                    return date_directory, paths
                sequence += 1
