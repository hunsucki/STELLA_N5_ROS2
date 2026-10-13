"""Load the installed STELLA DDS send guard before creating a ROS context."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, PackageNotFoundError


def _local_dds_environment(env):
    """Production Pi profile; never override custom or isolated DDS settings."""
    if (env.get('STELLA_DDS_LOOPBACK', '1') == '0'
            or env.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp'
            or env.get('ROS_DOMAIN_ID', '0') != '0'
            or env.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
            or env.get('ROS_LOCALHOST_ONLY', '0') == '1'
            or env.get('CYCLONEDDS_URI', '') not in (
                '', '/etc/cyclonedds.xml', 'file:///etc/cyclonedds.xml')):
        return
    try:
        profile = Path(get_package_prefix('stella_bringup'))/'share/stella_bringup/param/cyclonedds_local_control.xml'
    except PackageNotFoundError:
        return
    if profile.is_file():
        env['CYCLONEDDS_URI'] = profile.as_uri()


def guarded_environment(environment=None):
    env = dict(os.environ if environment is None else environment)
    _local_dds_environment(env)
    if env.get('DOCKING_DDS_UDP_GUARD', '1') == '0':
        return env, None
    try:
        library = Path(get_package_prefix('stella_bringup'))/'lib/libstella_dds_udp_guard.so'
    except PackageNotFoundError:
        return env, None
    if not library.is_file():
        return env, None
    library = str(library)
    # The loader accepts spaces and colons. Preserve any unrelated preload.
    entries = env.get('LD_PRELOAD', '').replace(':', ' ').split()
    if library not in entries:
        env['LD_PRELOAD'] = ' '.join([library, *entries])
    return env, library


def main():
    """Console entry: re-exec once so the loader installs the send guard."""
    env, library = guarded_environment()
    if any(env.get(name, '') != os.environ.get(name, '')
           for name in ('LD_PRELOAD', 'CYCLONEDDS_URI')):
        print('Loading STELLA DDS environment for docking and managed nodes', flush=True)
        os.execve(sys.executable, [sys.executable, *sys.argv], env)
    if library is None and os.environ.get('DOCKING_DDS_UDP_GUARD', '1') != '0':
        print('STELLA DDS UDP guard is unavailable',
              file=sys.stderr)
    from docking.dock_turn_backup import main as docking_main
    return docking_main()
