"""Versioned Basilisk-to-F Prime state packet.

The on-wire image is 256 bytes, little-endian, with no implicit C padding.
Python `struct` and `Orbiedge/TelemetryMonitor/BasiliskStatePacket.hpp` must
agree on every offset. Bump `VERSION` when a field moves.

ZMQ frame 0 is the topic `bsk.state`. Frame 1 is this packet.
The four-byte magic `BSK1` (0x42534B31) identifies the payload. It is not the topic.

Quaternion order is Basilisk `MRP2EP`: q0 is the scalar, then q1, q2, q3.
`bus_voltage_derived` is not a power-module measurement. Basilisk
`PowerStorageStatusMsgPayload` has charge and net power only. The bridge
computes a labeled stand-in voltage from state of charge.
"""

from __future__ import annotations

import struct
import zlib
from typing import NamedTuple, Sequence

MAGIC = 0x42534B31
VERSION = 1
PACKET_SIZE = 256
PAYLOAD_SIZE = 252
TOPIC = b"bsk.state"

FAULT_RW3_STICTION = 1 << 0
FAULT_BROWNOUT = 1 << 1
FAULT_IMU_WALK = 1 << 2

NOMINAL_BUS_VOLTAGE_V = 8.0

# 16-byte header + 26 little-endian doubles + sequence = 228, then 24 pad bytes.
_PREFIX = struct.Struct("<IHHQ4d3dddd3d3d4d4dddI")
_CRC = struct.Struct("<I")
assert _PREFIX.size == 228, _PREFIX.size
assert _PREFIX.size + 24 == PAYLOAD_SIZE


class BasiliskState(NamedTuple):
    sim_time_ns: int
    q_bn: tuple[float, float, float, float]
    omega_bn_b: tuple[float, float, float]
    bus_voltage_derived: float
    current_net_power_w: float
    soc: float
    wheel_speeds_rad_s: tuple[float, float, float]
    gyro_ang_vel_rad_s: tuple[float, float, float]
    star_tracker_q: tuple[float, float, float, float]
    css: tuple[float, float, float, float]
    panel_efficiency: float
    neutral_density_kg_m3: float
    fault_flags: int
    sequence: int


def derived_bus_voltage(soc: float) -> float:
    """Stand-in bus voltage from state of charge.

    V = 8.0 * (0.85 + 0.15 * clamp(soc, 0, 1)). An empty pack sits at 6.8 V
    and a full pack at 8.0 V. This is a bridge representation, not a cell model.
    """
    clamped = min(1.0, max(0.0, float(soc)))
    return NOMINAL_BUS_VOLTAGE_V * (0.85 + 0.15 * clamped)


def _four(values: Sequence[float], name: str) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError(f"{name} needs 4 values, got {len(values)}")
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))


def _three(values: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{name} needs 3 values, got {len(values)}")
    return (float(values[0]), float(values[1]), float(values[2]))


def pack_state(
    sim_time_ns: int,
    q_bn: Sequence[float],
    omega_bn_b: Sequence[float],
    bus_voltage_derived: float,
    current_net_power_w: float,
    soc: float,
    wheel_speeds_rad_s: Sequence[float],
    gyro_ang_vel_rad_s: Sequence[float],
    star_tracker_q: Sequence[float],
    css: Sequence[float],
    panel_efficiency: float,
    neutral_density_kg_m3: float,
    fault_flags: int,
    sequence: int,
) -> bytes:
    """Serialize one state sample. The return value is always 256 bytes."""
    attitude = _four(q_bn, "q_bn")
    rate = _three(omega_bn_b, "omega_bn_b")
    wheels = _three(wheel_speeds_rad_s, "wheel_speeds_rad_s")
    gyro = _three(gyro_ang_vel_rad_s, "gyro_ang_vel_rad_s")
    tracker = _four(star_tracker_q, "star_tracker_q")
    sun = _four(css, "css")
    prefix = _PREFIX.pack(
        MAGIC,
        VERSION,
        int(fault_flags) & 0xFFFF,
        int(sim_time_ns) & 0xFFFFFFFFFFFFFFFF,
        attitude[0],
        attitude[1],
        attitude[2],
        attitude[3],
        rate[0],
        rate[1],
        rate[2],
        float(bus_voltage_derived),
        float(current_net_power_w),
        float(soc),
        wheels[0],
        wheels[1],
        wheels[2],
        gyro[0],
        gyro[1],
        gyro[2],
        tracker[0],
        tracker[1],
        tracker[2],
        tracker[3],
        sun[0],
        sun[1],
        sun[2],
        sun[3],
        float(panel_efficiency),
        float(neutral_density_kg_m3),
        int(sequence) & 0xFFFFFFFF,
    )
    payload = prefix + bytes(24)
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    packet = payload + _CRC.pack(crc)
    if len(packet) != PACKET_SIZE:
        raise RuntimeError(f"packet length {len(packet)} != {PACKET_SIZE}")
    return packet


def unpack_state(packet: bytes) -> BasiliskState:
    """Decode one packet. Raises ValueError on size, magic, version, or CRC."""
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"size {len(packet)}")
    payload = packet[:PAYLOAD_SIZE]
    (expected_crc,) = _CRC.unpack_from(packet, PAYLOAD_SIZE)
    actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError("crc")
    fields = _PREFIX.unpack_from(payload, 0)
    magic = fields[0]
    version = fields[1]
    if magic != MAGIC:
        raise ValueError(f"magic 0x{magic:08X}")
    if version != VERSION:
        raise ValueError(f"version {version}")
    return BasiliskState(
        sim_time_ns=fields[3],
        q_bn=(fields[4], fields[5], fields[6], fields[7]),
        omega_bn_b=(fields[8], fields[9], fields[10]),
        bus_voltage_derived=fields[11],
        current_net_power_w=fields[12],
        soc=fields[13],
        wheel_speeds_rad_s=(fields[14], fields[15], fields[16]),
        gyro_ang_vel_rad_s=(fields[17], fields[18], fields[19]),
        star_tracker_q=(fields[20], fields[21], fields[22], fields[23]),
        css=(fields[24], fields[25], fields[26], fields[27]),
        panel_efficiency=fields[28],
        neutral_density_kg_m3=fields[29],
        fault_flags=fields[2],
        sequence=fields[30],
    )
