"""SIYI gimbal UDP packet encoding and transport."""

import math
import socket
import struct
import threading


def crc16_xmodem(data: bytes) -> int:
    """Return the SIYI CRC16-XMODEM checksum with an initial value of zero."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(
    command_id: int,
    payload: bytes = b'',
    sequence: int = 0,
    need_ack: bool = False,
) -> bytes:
    """Build one SIYI SDK frame with little-endian length, sequence, and CRC."""
    if not 0 <= command_id <= 0xFF:
        raise ValueError('command_id must be between 0 and 255')
    if len(payload) > 0xFFFF:
        raise ValueError('payload is too large')

    control = 0x01 if need_ack else 0x00
    frame = bytearray((0x55, 0x66, control))
    frame.extend(struct.pack('<H', len(payload)))
    frame.extend(struct.pack('<H', sequence & 0xFFFF))
    frame.append(command_id)
    frame.extend(payload)
    frame.extend(struct.pack('<H', crc16_xmodem(frame)))
    return bytes(frame)


def normalized_speed(value: float, direction: int = 1) -> int:
    """Convert a normalized ROS velocity to SIYI's signed -100..100 range."""
    if not math.isfinite(value):
        raise ValueError('velocity must be finite')
    if direction not in (-1, 1):
        raise ValueError('direction must be either -1 or 1')
    clamped = max(-1.0, min(1.0, float(value)))
    return int(round(clamped * 100.0)) * direction


class SiyiUdpClient:
    """Send sequenced SIYI control frames to one camera."""

    ROTATION_COMMAND = 0x07
    CENTER_COMMAND = 0x08
    ZOOM_COMMAND = 0x05

    def __init__(
        self,
        remote_address: str,
        remote_port: int = 37260,
        bind_address: str = '',
    ) -> None:
        """Create a UDP socket, optionally bound to a camera-link address."""
        self.remote = (remote_address, int(remote_port))
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if bind_address:
            self._socket.bind((bind_address, 0))
        self._sequence = 0
        self._lock = threading.Lock()

    def send_command(self, command_id: int, payload: bytes = b'') -> int:
        """Send a command without requesting an acknowledgement."""
        with self._lock:
            packet = build_packet(
                command_id,
                payload,
                sequence=self._sequence,
                need_ack=False,
            )
            sent = self._socket.sendto(packet, self.remote)
            self._sequence = (self._sequence + 1) & 0xFFFF
            return sent

    def rotate(self, yaw_speed: int, pitch_speed: int) -> int:
        """Send signed yaw and pitch speeds in the SIYI -100..100 range."""
        yaw = max(-100, min(100, int(yaw_speed)))
        pitch = max(-100, min(100, int(pitch_speed)))
        return self.send_command(
            self.ROTATION_COMMAND,
            struct.pack('<bb', yaw, pitch),
        )

    def zoom(self, direction: int) -> int:
        """Start zooming in/out, or stop zooming with zero."""
        value = max(-1, min(1, int(direction)))
        return self.send_command(self.ZOOM_COMMAND, struct.pack('<b', value))

    def center(self) -> int:
        """Move the gimbal to its center position."""
        return self.send_command(self.CENTER_COMMAND, b'\x01')

    def stop(self, repeats: int = 3) -> None:
        """Send redundant stop commands for both rotation and zoom."""
        for _ in range(max(1, repeats)):
            self.rotate(0, 0)
            self.zoom(0)

    def close(self) -> None:
        """Close the UDP socket."""
        self._socket.close()
