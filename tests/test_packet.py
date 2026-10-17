"""Round-trip the BSK1 packet with the standard library only.

Basilisk and pyzmq are intentionally not imported. Run from the repo root:

    python tests/test_packet.py
"""

from __future__ import annotations

import struct
import sys
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.bridge.packet import (  # noqa: E402
    FAULT_IMU_WALK,
    FAULT_RW3_STICTION,
    MAGIC,
    PACKET_SIZE,
    TOPIC,
    VERSION,
    derived_bus_voltage,
    pack_state,
    unpack_state,
)

GOLDEN = ROOT / "tests" / "golden" / "bsk_state_v1.bin"

SAMPLE = dict(
    sim_time_ns=120_000_000_000,
    q_bn=(0.5, 0.5, 0.5, 0.5),
    omega_bn_b=(0.125, -0.25, 0.5),
    bus_voltage_derived=8.0,
    current_net_power_w=-1.5,
    soc=0.75,
    wheel_speeds_rad_s=(1.0, 2.0, 3.0),
    gyro_ang_vel_rad_s=(0.25, 0.0, -0.25),
    star_tracker_q=(1.0, 0.0, 0.0, 0.0),
    css=(0.0, 1.0, 0.5, 0.25),
    panel_efficiency=0.125,
    neutral_density_kg_m3=0.0,
    fault_flags=FAULT_RW3_STICTION | FAULT_IMU_WALK,
    sequence=7,
)


class PacketTest(unittest.TestCase):
    def test_roundtrip_exact_doubles(self) -> None:
        packet = pack_state(**SAMPLE)
        self.assertEqual(len(packet), PACKET_SIZE)
        self.assertEqual(packet[:4], struct.pack("<I", MAGIC))
        self.assertEqual(struct.unpack_from("<H", packet, 4)[0], VERSION)
        state = unpack_state(packet)
        self.assertEqual(state.sim_time_ns, SAMPLE["sim_time_ns"])
        self.assertEqual(state.q_bn, SAMPLE["q_bn"])
        self.assertEqual(state.omega_bn_b, SAMPLE["omega_bn_b"])
        self.assertEqual(state.bus_voltage_derived, 8.0)
        self.assertEqual(state.current_net_power_w, -1.5)
        self.assertEqual(state.soc, 0.75)
        self.assertEqual(state.wheel_speeds_rad_s, (1.0, 2.0, 3.0))
        self.assertEqual(state.gyro_ang_vel_rad_s, (0.25, 0.0, -0.25))
        self.assertEqual(state.star_tracker_q, (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(state.css, (0.0, 1.0, 0.5, 0.25))
        self.assertEqual(state.panel_efficiency, 0.125)
        self.assertEqual(state.neutral_density_kg_m3, 0.0)
        self.assertEqual(state.fault_flags, SAMPLE["fault_flags"])
        self.assertEqual(state.sequence, 7)
        self.assertEqual(zlib.crc32(packet[:252]) & 0xFFFFFFFF, struct.unpack_from("<I", packet, 252)[0])

    def test_non_dyadic_values_roundtrip(self) -> None:
        packet = pack_state(
            sim_time_ns=1,
            q_bn=(0.9659258262890683, 0.0, 0.25881904510252074, 0.0),
            omega_bn_b=(0.001, -0.002, 0.003),
            bus_voltage_derived=derived_bus_voltage(0.9),
            current_net_power_w=-4.2,
            soc=0.9,
            wheel_speeds_rad_s=(10.5, -3.25, 0.125),
            gyro_ang_vel_rad_s=(0.0011, -0.0004, 0.0002),
            star_tracker_q=(0.98, 0.01, -0.02, 0.03),
            css=(0.2, 0.0, 0.7, 0.1),
            panel_efficiency=0.112,
            neutral_density_kg_m3=1.7e-12,
            fault_flags=FAULT_RW3_STICTION,
            sequence=3,
        )
        state = unpack_state(packet)
        self.assertAlmostEqual(state.bus_voltage_derived, derived_bus_voltage(0.9))
        self.assertAlmostEqual(state.neutral_density_kg_m3, 1.7e-12)
        self.assertAlmostEqual(state.q_bn[0], 0.9659258262890683)
        self.assertEqual(state.fault_flags, FAULT_RW3_STICTION)

    def test_crc_and_version_are_rejected(self) -> None:
        packet = bytearray(pack_state(**SAMPLE))
        packet[16] ^= 0x01
        with self.assertRaises(ValueError) as crc_error:
            unpack_state(bytes(packet))
        self.assertEqual(str(crc_error.exception), "crc")

        packet = bytearray(pack_state(**SAMPLE))
        struct.pack_into("<H", packet, 4, 99)
        struct.pack_into("<I", packet, 252, zlib.crc32(packet[:252]) & 0xFFFFFFFF)
        with self.assertRaises(ValueError) as version_error:
            unpack_state(bytes(packet))
        self.assertIn("version", str(version_error.exception))

    def test_voltage_curve(self) -> None:
        self.assertEqual(derived_bus_voltage(0.0), 6.8)
        self.assertEqual(derived_bus_voltage(1.0), 8.0)
        self.assertEqual(derived_bus_voltage(2.0), 8.0)
        self.assertEqual(derived_bus_voltage(-1.0), 6.8)

    def test_golden_file_matches(self) -> None:
        packet = pack_state(**SAMPLE)
        self.assertTrue(GOLDEN.is_file(), f"missing {GOLDEN}")
        self.assertEqual(GOLDEN.read_bytes(), packet)

    def test_topic_is_separate_from_magic(self) -> None:
        self.assertEqual(TOPIC, b"bsk.state")
        self.assertNotEqual(TOPIC, struct.pack("<I", MAGIC))


if __name__ == "__main__":
    unittest.main()
