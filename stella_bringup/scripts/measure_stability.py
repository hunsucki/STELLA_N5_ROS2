#!/usr/bin/env python3
"""Observe sensor latency and blocked threads without sending motor commands."""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu, JointState, LaserScan
from nav_msgs.msg import Odometry

try:
    import dbus
except ImportError:
    dbus = None

DRIVER_NAMES = {
    'stella_md_node', 'stella_ahrs_nod', 'wheel_odometry_',
    'sllidar_node', 'sllidar2_node',
}
TOPICS = {
    '/wheel/encoders': JointState,
    '/imu/data': Imu,
    '/odom': Odometry,
    '/scan': LaserScan,
    '/scan_2': LaserScan,
}


def wifi_properties():
    """Optional NetworkManager scan correlation; python3-dbus is not required."""
    if dbus is None:
        return None
    try:
        bus = dbus.SystemBus()
        manager = bus.get_object(
            'org.freedesktop.NetworkManager', '/org/freedesktop/NetworkManager')
        for device in manager.GetDevices(
                dbus_interface='org.freedesktop.NetworkManager'):
            properties = dbus.Interface(
                bus.get_object('org.freedesktop.NetworkManager', device),
                'org.freedesktop.DBus.Properties')
            if str(properties.Get(
                    'org.freedesktop.NetworkManager.Device', 'Interface')) == 'wlan0':
                return properties
    except dbus.DBusException:
        pass
    return None


def blocked_thread_states():
    """Linux comm is truncated to 15 characters; tolerate exits during reads."""
    states = {}
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            if (process / 'comm').read_text().strip() not in DRIVER_NAMES:
                continue
            states[process.name] = {
                thread.name: [
                    (thread / 'comm').read_text().strip(),
                    (thread / 'wchan').read_text().strip(),
                ]
                for thread in (process / 'task').iterdir()
            }
        except OSError:
            continue
    return states


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=145)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')

    rclpy.init()
    node = rclpy.create_node('bringup_stall_probe')
    wifi = wifi_properties()
    rows = {topic: [] for topic in TOPICS}
    last_receipt = {}
    events = []
    last_scan = None
    last_check = 0.0
    start = time.monotonic()
    subscriptions = []

    def callback(message, topic):
        now = time.monotonic()
        wall_time = time.time()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        gap = now - last_receipt[topic] if topic in last_receipt else 0.0
        # Row: wall receipt time, monotonic receipt gap, stamp age, stamp.
        rows[topic].append([wall_time, gap, wall_time - stamp, stamp])
        last_receipt[topic] = now

    for topic, message_type in TOPICS.items():
        subscriptions.append(node.create_subscription(
            message_type, topic,
            lambda message, topic=topic: callback(message, topic),
            qos_profile_sensor_data))

    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.03)
            now = time.monotonic()
            if now - last_check < 0.1:
                continue
            last_check = now
            if wifi is not None:
                try:
                    scan = int(wifi.Get(
                        'org.freedesktop.NetworkManager.Device.Wireless', 'LastScan'))
                    if scan != last_scan:
                        events.append({'time': time.time(), 'last_scan_ms': scan})
                        last_scan = scan
                except dbus.DBusException:
                    wifi = None
            gaps = {
                topic: round(now - receipt, 3)
                for topic, receipt in last_receipt.items() if now - receipt > 0.25
            }
            if gaps:
                event = {
                    'time': time.time(), 'gaps': gaps,
                    'states': blocked_thread_states(),
                }
                events.append(event)
                print(json.dumps(event), flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        summary = {}
        for topic, samples in rows.items():
            ages = sorted(sample[2] for sample in samples)
            summary[topic] = {
                'count': len(samples),
                'max_gap': max((sample[1] for sample in samples), default=0),
                'max_age': max(ages, default=0),
                'p95_age': ages[int(len(ages) * 0.95)] if ages else None,
            }
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({
            'duration_sec': time.monotonic() - start,
            'summary': summary, 'events': events, 'rows': rows,
        }, indent=2))
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
