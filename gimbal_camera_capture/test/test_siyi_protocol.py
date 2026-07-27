"""Tests for SIYI binary protocol encoding."""

import socket

from gimbal_camera_capture.siyi_protocol import (
    build_packet,
    normalized_speed,
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
