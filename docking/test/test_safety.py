from docking.safety import DockingExitCode, SingleInstanceLock


def test_single_instance_lock_rejects_duplicate_and_recovers(tmp_path):
    lock_path = str(tmp_path / 'dock.lock')
    first = SingleInstanceLock(lock_path)
    second = SingleInstanceLock(lock_path)

    assert first.acquire()
    assert not second.acquire()

    first.release()
    assert second.acquire()
    second.release()


def test_exit_code_contract():
    assert DockingExitCode.SUCCESS == 0
    assert all(code != 0 for code in DockingExitCode if code.name != 'SUCCESS')
