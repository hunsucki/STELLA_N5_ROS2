#!/usr/bin/env python3

"""Temporary BlueZ pairing agent for an Xbox controller."""

import signal
import sys

import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib


BLUEZ_SERVICE = 'org.bluez'
AGENT_INTERFACE = 'org.bluez.Agent1'
AGENT_MANAGER_INTERFACE = 'org.bluez.AgentManager1'
AGENT_PATH = '/org/stella/teleop_joy/agent'


class XboxPairingAgent(dbus.service.Object):

    def __init__(self, bus: dbus.SystemBus, loop: GLib.MainLoop) -> None:
        super().__init__(bus, AGENT_PATH)
        self._loop = loop

    @dbus.service.method(AGENT_INTERFACE, in_signature='', out_signature='')
    def Release(self) -> None:
        print('BlueZ released the pairing agent', flush=True)
        self._loop.quit()

    @dbus.service.method(AGENT_INTERFACE, in_signature='o', out_signature='s')
    def RequestPinCode(self, device: dbus.ObjectPath) -> str:
        print(f'Providing PIN 0000 for {device}', flush=True)
        return '0000'

    @dbus.service.method(AGENT_INTERFACE, in_signature='o', out_signature='u')
    def RequestPasskey(self, device: dbus.ObjectPath) -> dbus.UInt32:
        print(f'Providing passkey 000000 for {device}', flush=True)
        return dbus.UInt32(0)

    @dbus.service.method(AGENT_INTERFACE, in_signature='os', out_signature='')
    def DisplayPinCode(self, device: dbus.ObjectPath, pin_code: str) -> None:
        print(f'PIN for {device}: {pin_code}', flush=True)

    @dbus.service.method(AGENT_INTERFACE, in_signature='ouq', out_signature='')
    def DisplayPasskey(
        self,
        device: dbus.ObjectPath,
        passkey: dbus.UInt32,
        entered: dbus.UInt16,
    ) -> None:
        print(
            f'Passkey for {device}: {int(passkey):06d}, entered={int(entered)}',
            flush=True,
        )

    @dbus.service.method(AGENT_INTERFACE, in_signature='ou', out_signature='')
    def RequestConfirmation(
        self, device: dbus.ObjectPath, passkey: dbus.UInt32
    ) -> None:
        print(
            f'Automatically confirming passkey {int(passkey):06d} for {device}',
            flush=True,
        )

    @dbus.service.method(AGENT_INTERFACE, in_signature='o', out_signature='')
    def RequestAuthorization(self, device: dbus.ObjectPath) -> None:
        print(f'Authorizing pairing for {device}', flush=True)

    @dbus.service.method(AGENT_INTERFACE, in_signature='os', out_signature='')
    def AuthorizeService(self, device: dbus.ObjectPath, uuid: str) -> None:
        print(f'Authorizing service {uuid} for {device}', flush=True)

    @dbus.service.method(AGENT_INTERFACE, in_signature='', out_signature='')
    def Cancel(self) -> None:
        print('Pairing request cancelled', flush=True)


def main() -> int:
    DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    loop = GLib.MainLoop()
    agent = XboxPairingAgent(bus, loop)
    manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE, '/org/bluez'),
        AGENT_MANAGER_INTERFACE,
    )

    manager.RegisterAgent(AGENT_PATH, 'KeyboardDisplay')
    manager.RequestDefaultAgent(AGENT_PATH)

    def stop_agent(_signal_number, _frame) -> None:
        loop.quit()

    signal.signal(signal.SIGINT, stop_agent)
    signal.signal(signal.SIGTERM, stop_agent)
    print('READY: Xbox BlueZ pairing agent registered', flush=True)

    try:
        loop.run()
    finally:
        try:
            manager.UnregisterAgent(AGENT_PATH)
        except dbus.DBusException:
            pass
        # Keep the exported D-Bus object alive until it has been unregistered.
        del agent

    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except dbus.DBusException as error:
        print(f'BlueZ agent error: {error}', file=sys.stderr, flush=True)
        sys.exit(1)
