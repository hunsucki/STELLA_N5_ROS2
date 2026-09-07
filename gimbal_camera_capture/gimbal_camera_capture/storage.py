"""Persist run-scoped paths and metadata for paired camera captures."""

from datetime import datetime
import os
from pathlib import Path
import re
import threading
from typing import Dict, Iterable, Optional, Tuple

import yaml


class DatedCapturePaths:
    """Manage capture runs and their durable schema-v2 metadata."""

    METADATA_FILENAME = 'metadata.yaml'
    READY_FILENAME = 'READY'
    _RUN_PATTERN = re.compile(r'^run_(\d+)$')

    def __init__(self, base_directory: Path) -> None:
        """Initialize the run manager with an expanded absolute directory."""
        self._base_directory = base_directory.expanduser().resolve()
        self._reserved_paths = set()
        self._active_run_directory: Optional[Path] = None
        self._metadata: Optional[Dict] = None
        self._lock = threading.RLock()

    @property
    def base_directory(self) -> Path:
        """Return the capture base directory."""
        return self._base_directory

    @property
    def active_run_directory(self) -> Optional[Path]:
        """Return the active run directory, if a run is open."""
        with self._lock:
            return self._active_run_directory

    @property
    def active_run_id(self) -> str:
        """Return the active run name, or an empty string when idle."""
        with self._lock:
            if self._active_run_directory is None:
                return ''
            return self._active_run_directory.name

    @property
    def active_mission_id(self) -> str:
        """Return the mission owning the active run, if any."""
        with self._lock:
            if self._metadata is None:
                return ''
            return str(self._metadata.get('mission_id', ''))

    def start_run(
        self,
        timestamp: datetime,
        context: Optional[Dict] = None,
    ) -> Path:
        """Create and activate a new run for the timestamp's date."""
        with self._lock:
            if self._active_run_directory is not None:
                raise RuntimeError(
                    f'run already active: {self._active_run_directory.name}'
                )
            return self._start_run_locked(timestamp, context or {})

    def start_mission_run(
        self,
        timestamp: datetime,
        context: Dict,
    ) -> Tuple[Path, bool, str]:
        """Start a mission run or return its durable idempotent result."""
        mission_id = str(context.get('mission_id', ''))
        with self._lock:
            if self._metadata is not None:
                active_mission = str(self._metadata.get('mission_id', ''))
                if active_mission == mission_id:
                    return (
                        self._active_run_directory,
                        False,
                        str(self._metadata['status']),
                    )
                raise RuntimeError(
                    'another mission run is active: '
                    f'{active_mission or self.active_run_id}'
                )

            existing = self._find_run_by_mission_locked(mission_id)
            if existing is not None:
                run_directory, metadata = existing
                status = str(metadata.get('status', ''))
                if status == 'running':
                    self._activate_existing_locked(run_directory, metadata)
                return run_directory, False, status

            run_directory = self._start_run_locked(timestamp, context)
            return run_directory, True, 'running'

    def ensure_run(self, timestamp: datetime) -> Tuple[Path, bool]:
        """Return the active run, creating a legacy run when needed."""
        with self._lock:
            if self._active_run_directory is not None:
                return self._active_run_directory, False
            return self._start_run_locked(timestamp, {}), True

    def _start_run_locked(self, timestamp: datetime, context: Dict) -> Path:
        date_directory = self._base_directory / timestamp.strftime('%Y%m%d')
        date_directory.mkdir(parents=True, exist_ok=True)

        existing_numbers = []
        for path in date_directory.iterdir():
            match = self._RUN_PATTERN.fullmatch(path.name)
            if path.is_dir() and match:
                existing_numbers.append(int(match.group(1)))

        run_number = max(existing_numbers, default=0) + 1
        while True:
            run_directory = date_directory / f'run_{run_number}'
            try:
                run_directory.mkdir()
                break
            except FileExistsError:
                run_number += 1

        for camera in ('left', 'right'):
            (run_directory / camera).mkdir()

        self._active_run_directory = run_directory
        self._reserved_paths.clear()
        self._metadata = self._new_metadata(
            timestamp,
            run_directory.name,
            run_number,
            context,
        )
        try:
            self._write_metadata_locked()
        except Exception:
            self._clear_active_locked()
            raise
        return run_directory

    @staticmethod
    def _new_metadata(
        timestamp: datetime,
        run_id: str,
        run_number: int,
        context: Dict,
    ) -> Dict:
        zones = context.get('zones', {})
        return {
            'schema_version': 2,
            'run_id': run_id,
            'run_number': run_number,
            'mission_id': str(context.get('mission_id', '')),
            'date': timestamp.strftime('%Y%m%d'),
            'status': 'running',
            'started_at': timestamp.isoformat(),
            'finished_at': None,
            'abort_reason': '',
            'map': {
                'id': str(context.get('map_id', '')),
                'frame_id': str(context.get('map_frame', '')),
            },
            'zone_config': {
                'revision': str(context.get('zone_revision', '')),
                'assignment_rule': 'robot_base_at_request',
                'zones': dict(zones),
            },
            'cameras': ['left', 'right'],
            'summary': {
                'capture_requests': 0,
                'successful_pairs': 0,
                'partial_pairs': 0,
                'failed_requests': 0,
                'image_files': 0,
            },
            'captures': [],
        }

    def allocate(self, timestamp: datetime) -> Tuple[Path, Dict[str, Path]]:
        """Reserve one matching left/right filename in the active run."""
        with self._lock:
            if self._active_run_directory is None:
                self._start_run_locked(timestamp, {})
            run_directory = self._active_run_directory
            camera_directories = {
                'left': run_directory / 'left',
                'right': run_directory / 'right',
            }
            filename_prefix = timestamp.strftime('%H%M%S')

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
                    return run_directory, paths
                sequence += 1

    def validate_active_run(
        self,
        run_id: str,
        mission_id: str,
        zone_id: str = '',
    ) -> Dict:
        """Validate service identifiers and return active run metadata."""
        with self._lock:
            self._require_active_run_locked()
            self._validate_identifiers_locked(run_id, mission_id)
            if zone_id:
                zones = self._metadata['zone_config']['zones']
                if zone_id not in zones:
                    raise RuntimeError(f'unknown zone_id: {zone_id}')
            return dict(self._metadata)

    def find_capture(
        self,
        run_id: str,
        mission_id: str,
        request_id: str,
    ) -> Optional[Dict]:
        """Return an already recorded capture for idempotent retry."""
        with self._lock:
            found = self._find_matching_run_locked(run_id, mission_id)
            if found is None:
                return None
            _, metadata = found
            for capture in metadata.get('captures', []):
                if capture.get('request_id') == request_id:
                    return dict(capture)
            return None

    def record_capture(
        self,
        timestamp: datetime,
        files: Dict[str, str],
        errors: Dict[str, str],
        success: bool,
    ) -> Path:
        """Append one legacy capture request to the active metadata."""
        metadata_path, _ = self.record_service_capture(
            captured_at=timestamp,
            files=files,
            errors=errors,
            success=success,
        )
        return metadata_path

    def record_service_capture(
        self,
        captured_at: datetime,
        files: Dict[str, str],
        errors: Dict[str, str],
        success: bool,
        request_id: str = '',
        zone_id: str = '',
        requested_at: str = '',
        robot_pose: Optional[Dict] = None,
    ) -> Tuple[Path, Dict]:
        """Atomically append one service capture and return its record."""
        with self._lock:
            self._require_active_run_locked()
            if request_id:
                for existing in self._metadata['captures']:
                    if existing.get('request_id') == request_id:
                        return self._metadata_path_locked(), dict(existing)

            relative_files = {
                camera: self._relative_path_locked(path)
                for camera, path in files.items()
            }
            captures = self._metadata['captures']
            capture = {
                'capture_id': len(captures) + 1,
                'request_id': request_id,
                'zone_id': zone_id,
                'requested_at': requested_at or captured_at.isoformat(),
                'captured_at': captured_at.isoformat(),
                'robot_pose': dict(robot_pose or {}),
                'success': bool(success),
                'files': relative_files,
                'errors': dict(errors),
            }
            captures.append(capture)

            summary = self._metadata['summary']
            summary['capture_requests'] += 1
            summary['image_files'] += len(relative_files)
            if success:
                summary['successful_pairs'] += 1
            elif relative_files:
                summary['partial_pairs'] += 1
            else:
                summary['failed_requests'] += 1

            return self._write_metadata_locked(), dict(capture)

    def finish_run(self, timestamp: datetime) -> Tuple[Path, Path]:
        """Finalize the active legacy run and mark it ready for transfer."""
        with self._lock:
            self._require_active_run_locked()
            return self._finish_active_locked(timestamp)

    def finish_mission_run(
        self,
        timestamp: datetime,
        run_id: str,
        mission_id: str,
    ) -> Tuple[Path, Path, bool]:
        """Finish a mission run, including idempotent finish retries."""
        with self._lock:
            found = self._find_matching_run_locked(run_id, mission_id)
            if found is None:
                raise RuntimeError('run_id and mission_id do not match a run')
            run_directory, metadata = found
            status = str(metadata.get('status', ''))
            if status == 'aborted':
                raise RuntimeError('cannot finish an aborted run')
            if status == 'completed':
                ready_path = run_directory / self.READY_FILENAME
                if not ready_path.is_file():
                    self._write_ready(run_directory, timestamp)
                return (
                    run_directory,
                    run_directory / self.METADATA_FILENAME,
                    True,
                )
            if status != 'running':
                raise RuntimeError(f'run has invalid status: {status}')

            if self._active_run_directory is None:
                self._activate_existing_locked(run_directory, metadata)
            elif self._active_run_directory != run_directory:
                raise RuntimeError('another run is active')
            directory, metadata_path = self._finish_active_locked(timestamp)
            return directory, metadata_path, False

    def _finish_active_locked(
        self,
        timestamp: datetime,
    ) -> Tuple[Path, Path]:
        self._require_active_run_locked()
        run_directory = self._active_run_directory
        self._metadata['status'] = 'completed'
        self._metadata['finished_at'] = timestamp.isoformat()
        try:
            metadata_path = self._write_metadata_locked()
            self._write_ready(run_directory, timestamp)
        except Exception:
            self._metadata['status'] = 'running'
            self._metadata['finished_at'] = None
            self._write_metadata_locked()
            raise
        self._clear_active_locked()
        return run_directory, metadata_path

    def abort_mission_run(
        self,
        timestamp: datetime,
        run_id: str,
        mission_id: str,
        reason: str,
    ) -> Tuple[Path, Path, bool]:
        """Abort a mission run without creating a READY marker."""
        with self._lock:
            found = self._find_matching_run_locked(run_id, mission_id)
            if found is None:
                raise RuntimeError('run_id and mission_id do not match a run')
            run_directory, metadata = found
            status = str(metadata.get('status', ''))
            if status == 'aborted':
                (run_directory / self.READY_FILENAME).unlink(missing_ok=True)
                return (
                    run_directory,
                    run_directory / self.METADATA_FILENAME,
                    True,
                )
            if status == 'completed':
                raise RuntimeError('cannot abort a completed run')
            if status != 'running':
                raise RuntimeError(f'run has invalid status: {status}')

            if self._active_run_directory is None:
                self._activate_existing_locked(run_directory, metadata)
            elif self._active_run_directory != run_directory:
                raise RuntimeError('another run is active')

            self._metadata['status'] = 'aborted'
            self._metadata['finished_at'] = timestamp.isoformat()
            self._metadata['abort_reason'] = reason
            metadata_path = self._write_metadata_locked()
            (run_directory / self.READY_FILENAME).unlink(missing_ok=True)
            self._clear_active_locked()
            return run_directory, metadata_path, False

    def _validate_identifiers_locked(
        self,
        run_id: str,
        mission_id: str,
    ) -> None:
        if self.active_run_id != run_id:
            raise RuntimeError(
                f'run_id mismatch: active={self.active_run_id!r}, '
                f'requested={run_id!r}'
            )
        active_mission = str(self._metadata.get('mission_id', ''))
        if active_mission != mission_id:
            raise RuntimeError(
                f'mission_id mismatch: active={active_mission!r}'
            )

    def _find_matching_run_locked(
        self,
        run_id: str,
        mission_id: str,
    ) -> Optional[Tuple[Path, Dict]]:
        if self._metadata is not None:
            if (
                self.active_run_id == run_id
                and self._metadata.get('mission_id') == mission_id
            ):
                return self._active_run_directory, self._metadata

        existing = self._find_run_by_mission_locked(mission_id)
        if existing is None or existing[0].name != run_id:
            return None
        return existing

    def _find_run_by_mission_locked(
        self,
        mission_id: str,
    ) -> Optional[Tuple[Path, Dict]]:
        if not mission_id:
            return None
        for metadata_path in self._metadata_paths_locked():
            metadata = self._read_metadata(metadata_path)
            if metadata.get('mission_id') == mission_id:
                return metadata_path.parent, metadata
        return None

    def _metadata_paths_locked(self) -> Iterable[Path]:
        if not self._base_directory.exists():
            return []
        paths = self._base_directory.glob(
            '[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]/'
            'run_*/metadata.yaml'
        )
        return sorted(paths, reverse=True)

    @staticmethod
    def _read_metadata(metadata_path: Path) -> Dict:
        try:
            with metadata_path.open(encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            raise RuntimeError(
                f'cannot read metadata {metadata_path}: {error}'
            ) from error
        if not isinstance(metadata, dict):
            raise RuntimeError(f'invalid metadata: {metadata_path}')
        return metadata

    def _activate_existing_locked(
        self,
        run_directory: Path,
        metadata: Dict,
    ) -> None:
        self._active_run_directory = run_directory
        self._metadata = metadata
        self._reserved_paths.clear()

    def _clear_active_locked(self) -> None:
        self._active_run_directory = None
        self._metadata = None
        self._reserved_paths.clear()

    def _require_active_run_locked(self) -> None:
        if self._active_run_directory is None or self._metadata is None:
            raise RuntimeError('no active run')

    def _relative_path_locked(self, path: str) -> str:
        candidate = Path(path).expanduser().resolve()
        try:
            return str(candidate.relative_to(self._active_run_directory))
        except ValueError:
            return str(candidate)

    def _metadata_path_locked(self) -> Path:
        self._require_active_run_locked()
        return self._active_run_directory / self.METADATA_FILENAME

    def _write_metadata_locked(self) -> Path:
        metadata_path = self._metadata_path_locked()
        contents = yaml.safe_dump(
            self._metadata,
            allow_unicode=True,
            sort_keys=False,
        )
        self._atomic_write(metadata_path, contents)
        return metadata_path

    def _write_ready(self, run_directory: Path, timestamp: datetime) -> None:
        ready_path = run_directory / self.READY_FILENAME
        self._atomic_write(
            ready_path,
            f'completed_at: {timestamp.isoformat()}\n',
        )

    @staticmethod
    def _atomic_write(path: Path, contents: str) -> None:
        temporary_path = path.with_name(f'.{path.name}.tmp')
        with temporary_path.open('w', encoding='utf-8') as stream:
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
