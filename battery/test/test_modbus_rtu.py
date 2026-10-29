"""Reject valid-CRC responses that belong to a different request."""

import struct
from unittest.mock import Mock

from battery.modbus_rtu import _build_frame, _crc16, ModbusRTU
import pytest


def client(response):
    bus = ModbusRTU.__new__(ModbusRTU)
    bus.slave_id = 1
    bus._ser = Mock()
    bus._ser.read.return_value = response
    return bus


def test_known_crc_vector():
    assert _crc16(bytes.fromhex('01030000000a')) == 0xcdc5


@pytest.mark.parametrize('slave, count', [(2, 2), (1, 0)])
def test_read_rejects_wrong_slave_and_byte_count(slave, count):
    response = _build_frame(slave, 3, bytes([count, 0, 10]))
    assert client(response).read_registers(0, 1) is None


def test_read_valid_frame_and_crc_failure():
    response = _build_frame(1, 3, bytes([2, 0, 10]))
    assert client(response).read_register(0) == 10
    assert client(response[:-1] + bytes([response[-1] ^ 1])).read_register(0) is None


@pytest.mark.parametrize('slave, address, value', [(2, 1, 100), (1, 2, 100), (1, 1, 200)])
def test_write_requires_exact_echo(slave, address, value):
    response = _build_frame(slave, 6, struct.pack('>HH', address, value))
    assert not client(response).write_register(1, 100)


def test_write_valid_echo():
    response = _build_frame(1, 6, struct.pack('>HH', 1, 100))
    assert client(response).write_register(1, 100)
