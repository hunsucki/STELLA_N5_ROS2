"""Tests for SIYI binary protocol encoding."""

import socket
import struct
import threading

from gimbal_camera_capture.siyi_protocol import (
    build_packet,
    normalized_speed,
    parse_packet,
    SiyiUdpClient,
)
import pytest


def test_center_packet_matches_official_example():
    """Center framing and CRC match the SIYI SDK manual example."""
    packet = build_packet(0x08, b'\x01', sequence=0, need_ack=True)

    assert packet == bytes.fromhex(
        '55 66 01 01 00 00 00 08 01 d1 12'
    )


def test_normalized_speed_clamps_and_applies_direction():
    """ROS normalized inputs map safely into the signed SIYI range."""
    assert normalized_speed(0.4) == 40
    assert normalized_speed(2.0) == 100
    assert normalized_speed(-2.0) == -100
    assert normalized_speed(0.4, -1) == -40

    with pytest.raises(ValueError):
        normalized_speed(float('nan'))
    with pytest.raises(ValueError):
        normalized_speed(0.5, 0)


def test_udp_client_sends_rotation_packet():
    """The client sends yaw and pitch as two signed bytes."""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    server.settimeout(1.0)
    client = SiyiUdpClient('127.0.0.1', server.getsockname()[1])

    try:
        client.rotate(-40, 25)
        packet, _ = server.recvfrom(64)
    finally:
        client.close()
        server.close()

    expected = build_packet(0x07, bytes((216, 25)), sequence=0)
    assert packet == expected


def test_udp_client_sends_absolute_a8_mini_angles():
    """Absolute angles use signed little-endian tenths of a degree."""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    server.settimeout(1.0)
    client = SiyiUdpClient('127.0.0.1', server.getsockname()[1])

    try:
        client.set_angles(-45.5, -30.0)
        packet, _ = server.recvfrom(64)
    finally:
        client.close()
        server.close()

    payload = struct.pack('<hh', -455, -300)
    assert packet == build_packet(0x0E, payload, sequence=0)


def test_udp_client_requires_valid_ack_for_reliable_absolute_angles():
    """Reliable startup commands request and validate the camera ACK."""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    server.settimeout(1.0)
    received = {}

    def acknowledge():
        packet, peer = server.recvfrom(64)
        received['packet'] = packet
        response_payload = struct.pack('<hhh', -400, -300, 0)
        server.sendto(
            build_packet(
                0x0E,
                response_payload,
                sequence=12,
                ack_packet=True,
            ),
            peer,
        )

    responder = threading.Thread(target=acknowledge)
    responder.start()
    client = SiyiUdpClient('127.0.0.1', server.getsockname()[1])
    try:
        response = client.set_angles_and_wait(
            -40.0,
            -30.0,
            timeout_sec=0.5,
            retries=1,
        )
    finally:
        responder.join(timeout=1.0)
        client.close()
        server.close()

    control, _, command_id, payload = parse_packet(received['packet'])
    assert control == 0x01
    assert command_id == 0x0E
    assert payload == struct.pack('<hh', -400, -300)
    assert response == struct.pack('<hhh', -400, -300, 0)


def test_parse_packet_rejects_corrupt_crc():
    """A corrupt datagram cannot satisfy reliable command handling."""
    packet = bytearray(build_packet(0x0E, b'\x00\x00\x00\x00'))
    packet[-1] ^= 0xFF

    with pytest.raises(ValueError, match='CRC'):
        parse_packet(bytes(packet))


@pytest.mark.parametrize(
    ('yaw', 'pitch'),
    [(136.0, 0.0), (-136.0, 0.0), (0.0, 26.0), (0.0, -91.0)],
)
def test_absolute_angles_reject_values_outside_a8_mini_limits(
    yaw,
    pitch,
):
    """A8 Mini mechanical limits reject unsafe absolute targets."""
    client = SiyiUdpClient('127.0.0.1', 37260)
    try:
        with pytest.raises(ValueError):
            client.set_angles(yaw, pitch)
    finally:
        client.close()
