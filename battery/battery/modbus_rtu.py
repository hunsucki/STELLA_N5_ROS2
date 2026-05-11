#!/usr/bin/env python3
"""
Minimal Modbus RTU implementation over pyserial.

Supports FC03 (Read Holding Registers) and FC06 (Write Single Register).
"""

import struct
import time

import serial


def _crc16(data: bytes) -> int:
    """Calculate Modbus CRC16."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _build_frame(slave_id: int, function_code: int, payload: bytes) -> bytes:
    body = bytes([slave_id, function_code]) + payload
    crc = _crc16(body)
    return body + struct.pack('<H', crc)


def _check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    crc_received = struct.unpack('<H', frame[-2:])[0]
    crc_calc = _crc16(frame[:-2])
    return crc_received == crc_calc


class ModbusRTU:
    """
    Simple Modbus RTU master.

    Parameters
    ----------
    port      : serial port path, e.g. '/dev/ttyUSB0'
    baudrate  : baud rate (XY-SK120 default is 9600)
    slave_id  : Modbus slave address (default 1)
    timeout   : serial read timeout in seconds

    """

    def __init__(self, port: str, baudrate: int = 9600,
                 slave_id: int = 1, timeout: float = 1.0):
        self.slave_id = slave_id
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        # Let the line settle
        time.sleep(0.1)

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    # FC03: Read Holding Registers
    # ------------------------------------------------------------------
    def read_register(self, address: int) -> int | None:
        """Read a single holding register. Returns raw uint16 or None on error."""
        result = self.read_registers(address, 1)
        if result is not None:
            return result[0]
        return None

    def read_registers(self, start_address: int, count: int) -> list[int] | None:
        """
        Read *count* holding registers starting at *start_address*.

        Returns list of uint16 values or None on error.
        """
        payload = struct.pack('>HH', start_address, count)
        frame = _build_frame(self.slave_id, 0x03, payload)
        self._ser.reset_input_buffer()
        self._ser.write(frame)

        # Expected response: addr(1) + FC(1) + byte_count(1) + data(count*2) + CRC(2)
        expected = 3 + count * 2 + 2
        response = self._ser.read(expected)

        if len(response) != expected:
            return None
        if not _check_crc(response):
            return None
        if response[1] != 0x03:
            return None

        values = []
        for i in range(count):
            hi = response[3 + i * 2]
            lo = response[4 + i * 2]
            values.append((hi << 8) | lo)
        return values

    # ------------------------------------------------------------------
    # FC06: Write Single Register
    # ------------------------------------------------------------------
    def write_register(self, address: int, value: int) -> bool:
        """Write a single holding register. Returns True on success."""
        payload = struct.pack('>HH', address, value)
        frame = _build_frame(self.slave_id, 0x06, payload)
        self._ser.reset_input_buffer()
        self._ser.write(frame)

        # Response is an echo of the request (8 bytes)
        response = self._ser.read(8)
        if len(response) != 8:
            return False
        if not _check_crc(response):
            return False
        if response[1] != 0x06:
            return False
        return True
