"""Process-level safety helpers for the remote docking command."""

import ctypes
from enum import IntEnum
import fcntl
import os
import signal


class DockingExitCode(IntEnum):
    SUCCESS = 0
    INTERNAL_ERROR = 1
    INVALID_REQUEST = 2
    SENSOR_OR_BASE_UNAVAILABLE = 3
    DOCKING_FAILED = 4
    TIMEOUT = 5
    CHARGING_NOT_CONFIRMED = 6
    SIGHUP = 129
    SIGINT = 130
    SIGTERM = 143


class SingleInstanceLock:
    """Advisory lock that is released automatically when the process exits."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.fd: int | None = None

    def acquire(self) -> bool:
        flags = os.O_CREAT | os.O_RDWR
        if hasattr(os, 'O_NOFOLLOW'):
            flags |= os.O_NOFOLLOW

        self.fd = os.open(self.path, flags, 0o600)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self.fd)
            self.fd = None
            return False

        os.ftruncate(self.fd, 0)
        os.write(self.fd, f'{os.getpid()}\n'.encode('ascii'))
        return True

    def release(self) -> None:
        if self.fd is None:
            return
        try:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
        finally:
            os.close(self.fd)
            self.fd = None


def install_parent_death_signal(signum: int = signal.SIGTERM) -> bool:
    """Ask Linux to signal this process when its direct parent exits."""
    pr_set_pdeathsig = 1
    parent_pid = os.getppid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(pr_set_pdeathsig, signum, 0, 0, 0) != 0:
        return False

    # Close the race where the parent exits immediately before prctl().
    if os.getppid() != parent_pid:
        os.kill(os.getpid(), signum)
    return True
