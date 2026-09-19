"""SIYI gimbal UDP packet encoding and transport."""

import math
import socket
import struct
import threading
import time


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
    ack_packet: bool = False,
) -> bytes:
    """Build one SIYI SDK frame with little-endian length, sequence, and CRC."""
    if not 0 <= command_id <= 0xFF:
        raise ValueError('command_id must be between 0 and 255')
    if len(payload) > 0xFFFF:
        raise ValueError('payload is too large')
    if need_ack and ack_packet:
        raise ValueError('a packet cannot request and be an acknowledgement')

    control = 0x02 if ack_packet else (0x01 if need_ack else 0x00)
    frame = bytearray((0x55, 0x66, control))
    frame.extend(struct.pack('<H', len(payload)))
    frame.extend(struct.pack('<H', sequence & 0xFFFF))
    frame.append(command_id)
    frame.extend(payload)
    frame.extend(struct.pack('<H', crc16_xmodem(frame)))
    return bytes(frame)


def parse_packet(packet: bytes):
    """
    Validate and split one SIYI SDK frame.

    Return ``(control, sequence, command_id, payload)``.  Invalid or
    truncated datagrams raise ``ValueError`` instead of being mistaken for
    acknowledgements.
    """
    if len(packet) < 10:
        raise ValueError('SIYI packet is shorter than the 10-byte minimum')
    if packet[:2] != b'\x55\x66':
        raise ValueError('SIYI packet has an invalid start marker')

    payload_length = struct.unpack_from('<H', packet, 3)[0]
    expected_length = 10 + payload_length
    if len(packet) != expected_length:
        raise ValueError(
            'SIYI packet length does not match its data length'
        )

    received_crc = struct.unpack_from('<H', packet, len(packet) - 2)[0]
    expected_crc = crc16_xmodem(packet[:-2])
    if received_crc != expected_crc:
        raise ValueError('SIYI packet CRC is invalid')

    sequence = struct.unpack_from('<H', packet, 5)[0]
    return packet[2], sequence, packet[7], packet[8:-2]


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
    SET_ANGLE_COMMAND = 0x0E
    ZOOM_COMMAND = 0x05

    A8_MIN_YAW_DEG = -135.0
    A8_MAX_YAW_DEG = 135.0
    A8_MIN_PITCH_DEG = -90.0
    A8_MAX_PITCH_DEG = 25.0

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

    def send_command_with_ack(
        self,
        command_id: int,
        payload: bytes = b'',
        timeout_sec: float = 0.5,
        retries: int = 3,
    ) -> bytes:
        """Send a command and return its validated acknowledgement payload."""
        timeout = float(timeout_sec)
        attempts = int(retries)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('timeout_sec must be a positive finite value')
        if attempts < 1:
            raise ValueError('retries must be at least 1')

        with self._lock:
            previous_timeout = self._socket.gettimeout()
            try:
                for _ in range(attempts):
                    packet = build_packet(
                        command_id,
                        payload,
                        sequence=self._sequence,
                        need_ack=True,
                    )
                    self._socket.sendto(packet, self.remote)
                    self._sequence = (self._sequence + 1) & 0xFFFF
                    deadline = time.monotonic() + timeout

                    while True:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            break
                        self._socket.settimeout(remaining)
                        try:
                            response, peer = self._socket.recvfrom(65535)
                        except socket.timeout:
                            break
                        if peer != self.remote:
                            continue
                        try:
                            control, _, response_command, data = (
                                parse_packet(response)
                            )
                        except ValueError:
                            continue
                        if (
                            control & 0x02
                            and response_command == command_id
                        ):
                            return data
            finally:
                self._socket.settimeout(previous_timeout)

        raise TimeoutError(
            f'no valid SIYI acknowledgement for command '
            f'0x{command_id:02X} from {self.remote[0]}:{self.remote[1]} '
            f'after {attempts} attempts'
        )

    def route_source_address(self) -> str:
        """Return the IPv4 source address selected for this camera route."""
        bound_address = str(self._socket.getsockname()[0])
        if bound_address != '0.0.0.0':
            return bound_address
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(self.remote)
            return str(probe.getsockname()[0])
        finally:
            probe.close()

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

    def center_and_wait(
        self,
        timeout_sec: float = 0.5,
        retries: int = 3,
    ) -> bytes:
        """Center the gimbal and require a valid camera acknowledgement."""
        return self.send_command_with_ack(
            self.CENTER_COMMAND,
            b'\x01',
            timeout_sec,
            retries,
        )

    def _angle_payload(self, yaw_deg: float, pitch_deg: float) -> bytes:
        """Validate and encode A8 Mini absolute angles."""
        yaw = float(yaw_deg)
        pitch = float(pitch_deg)
        if not math.isfinite(yaw) or not math.isfinite(pitch):
            raise ValueError('gimbal angles must be finite')
        if not self.A8_MIN_YAW_DEG <= yaw <= self.A8_MAX_YAW_DEG:
            raise ValueError(
                'yaw angle must be between -135.0 and 135.0 degrees'
            )
        if not self.A8_MIN_PITCH_DEG <= pitch <= self.A8_MAX_PITCH_DEG:
            raise ValueError(
                'pitch angle must be between -90.0 and 25.0 degrees'
            )
        return struct.pack(
            '<hh',
            int(round(yaw * 10.0)),
            int(round(pitch * 10.0)),
        )

    def set_angles(self, yaw_deg: float, pitch_deg: float) -> int:
        """Move an A8 Mini to absolute yaw and pitch angles in degrees."""
        return self.send_command(
            self.SET_ANGLE_COMMAND,
            self._angle_payload(yaw_deg, pitch_deg),
        )

    def set_angles_and_wait(
        self,
        yaw_deg: float,
        pitch_deg: float,
        timeout_sec: float = 0.5,
        retries: int = 3,
    ) -> bytes:
        """Set absolute angles and require a valid camera acknowledgement."""
        return self.send_command_with_ack(
            self.SET_ANGLE_COMMAND,
            self._angle_payload(yaw_deg, pitch_deg),
            timeout_sec,
            retries,
        )

    def stop(self, repeats: int = 3) -> None:
        """Send redundant stop commands for both rotation and zoom."""
        for _ in range(max(1, repeats)):
            self.rotate(0, 0)
            self.zoom(0)

    def close(self) -> None:
        """Close the UDP socket."""
        self._socket.close()
