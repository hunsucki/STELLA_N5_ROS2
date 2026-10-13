"""SSH-launched docking must carry bringup's send guard into its own stack."""

from unittest.mock import Mock

from docking import runtime
import pytest


def guard_file(tmp_path, monkeypatch):
    library = tmp_path/'lib/libstella_dds_udp_guard.so'
    library.parent.mkdir()
    library.touch()
    monkeypatch.setattr(runtime, 'get_package_prefix', lambda _: str(tmp_path))
    return str(library)


def test_guard_preserves_existing_environment_and_is_idempotent(tmp_path, monkeypatch):
    library = guard_file(tmp_path, monkeypatch)
    original = {'LD_PRELOAD': '/existing/a.so:/existing/b.so', 'ROS_DOMAIN_ID': '217'}
    env, selected = runtime.guarded_environment(original)
    assert selected == library
    assert env['LD_PRELOAD'].split() == [library, '/existing/a.so', '/existing/b.so']
    assert env['ROS_DOMAIN_ID'] == '217'
    assert original['LD_PRELOAD'] == '/existing/a.so:/existing/b.so'
    assert runtime.guarded_environment(env) == (env, library)


@pytest.mark.parametrize('reason', ['disabled', 'missing_file', 'missing_package'])
def test_unavailable_or_disabled_guard_preserves_environment(tmp_path, monkeypatch, reason):
    monkeypatch.setattr(runtime, 'get_package_prefix', lambda _: str(tmp_path))
    env = {'ROS_DOMAIN_ID': '217'}
    if reason == 'disabled':
        env['DOCKING_DDS_UDP_GUARD'] = '0'
    elif reason == 'missing_package':
        def missing(_):
            raise runtime.PackageNotFoundError('stella_bringup')
        monkeypatch.setattr(runtime, 'get_package_prefix', missing)
    assert runtime.guarded_environment(env) == (env, None)


def test_console_reexec_preserves_cli_before_ros_initialization(tmp_path, monkeypatch):
    library = guard_file(tmp_path, monkeypatch)
    monkeypatch.delenv('LD_PRELOAD', raising=False)
    monkeypatch.delenv('DOCKING_DDS_UDP_GUARD', raising=False)
    arguments = ['/installed/dock_turn_backup', '--ros-args', '-p', 'development_test_mode:=true']
    monkeypatch.setattr(runtime.sys, 'argv', arguments)
    class Reexecuted(Exception):
        pass
    execute = Mock(side_effect=Reexecuted)
    monkeypatch.setattr(runtime.os, 'execve', execute)
    with pytest.raises(Reexecuted):
        runtime.main()
    executable, argv, env = execute.call_args.args
    assert executable == runtime.sys.executable
    assert argv == [executable, *arguments]
    assert env['LD_PRELOAD'].split()[0] == library


def test_reexecuted_console_enters_docking_once_without_recursion(tmp_path, monkeypatch):
    from docking import dock_turn_backup
    library = guard_file(tmp_path, monkeypatch)
    monkeypatch.setenv('LD_PRELOAD', library)
    monkeypatch.delenv('DOCKING_DDS_UDP_GUARD', raising=False)
    execute = Mock()
    monkeypatch.setattr(runtime.os, 'execve', execute)
    entry = Mock(return_value=4)
    monkeypatch.setattr(dock_turn_backup, 'main', entry)
    assert runtime.main() == 4
    execute.assert_not_called()
    entry.assert_called_once_with()


def test_managed_process_inherits_guard_and_existing_environment(tmp_path, monkeypatch):
    from docking.stack_manager import ManagedStack
    from types import SimpleNamespace
    library = guard_file(tmp_path, monkeypatch)
    monkeypatch.delenv('DOCKING_DDS_UDP_GUARD', raising=False)
    monkeypatch.setenv('LD_PRELOAD', '/existing.so')
    monkeypatch.setenv('ROS_DOMAIN_ID', '217')
    popen = Mock()
    monkeypatch.setattr('docking.stack_manager.subprocess.Popen', popen)
    stack = ManagedStack.__new__(ManagedStack)
    stack.node = SimpleNamespace(get_logger=lambda: Mock())
    stack.processes, stack._launch_pids = [], set()
    stack._start_process('rectifier', ['/usr/bin/ros2', 'run', 'docking', 'image_rectifier'])
    env = popen.call_args.kwargs['env']
    assert env['LD_PRELOAD'].split() == [library, '/existing.so']
    assert env['ROS_DOMAIN_ID'] == '217'
    assert popen.call_args.kwargs['start_new_session'] is True


def local_profile(tmp_path, monkeypatch):
    guard_file(tmp_path, monkeypatch)
    profile = tmp_path/'share/stella_bringup/param/cyclonedds_local_control.xml'
    profile.parent.mkdir(parents=True)
    profile.write_text('<CycloneDDS/>')
    return profile.as_uri()


def test_ssh_production_environment_selects_same_profile_once(tmp_path, monkeypatch):
    profile = local_profile(tmp_path, monkeypatch)
    original = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'ROS_DOMAIN_ID': '0',
                'CYCLONEDDS_URI': 'file:///etc/cyclonedds.xml'}
    env, library = runtime.guarded_environment(original)
    assert env['CYCLONEDDS_URI'] == profile
    assert original['CYCLONEDDS_URI'] == 'file:///etc/cyclonedds.xml'
    assert runtime.guarded_environment(env) == (env, library)


@pytest.mark.parametrize('setting,value', [
    ('CYCLONEDDS_URI', 'file:///custom.xml'),
    ('CYCLONEDDS_URI', '<CycloneDDS><Domain/></CycloneDDS>'),
    ('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
    ('ROS_DOMAIN_ID', '217'),
    ('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
    ('ROS_LOCALHOST_ONLY', '1'),
    ('STELLA_DDS_LOOPBACK', '0')])
def test_profile_preserves_custom_transport_and_ros_isolation(tmp_path, monkeypatch, setting, value):
    local_profile(tmp_path, monkeypatch)
    original = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                'CYCLONEDDS_URI': 'file:///etc/cyclonedds.xml', setting: value}
    env, _ = runtime.guarded_environment(original)
    assert env.get('CYCLONEDDS_URI') == original.get('CYCLONEDDS_URI')


def test_already_guarded_console_reexecutes_for_dds_profile_before_ros(tmp_path, monkeypatch):
    profile = local_profile(tmp_path, monkeypatch)
    library = str(tmp_path/'lib/libstella_dds_udp_guard.so')
    monkeypatch.setenv('LD_PRELOAD', library)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('CYCLONEDDS_URI', 'file:///etc/cyclonedds.xml')
    for name in ('STELLA_DDS_LOOPBACK', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_LOCALHOST_ONLY'):
        monkeypatch.delenv(name, raising=False)
    class Reexecuted(Exception):
        pass
    execute = Mock(side_effect=Reexecuted)
    monkeypatch.setattr(runtime.os, 'execve', execute)
    with pytest.raises(Reexecuted):
        runtime.main()
    assert execute.call_args.args[2]['CYCLONEDDS_URI'] == profile
