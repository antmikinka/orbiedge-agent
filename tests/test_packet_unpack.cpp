// Framework-free check of BasiliskStatePacket.hpp against tests/golden/bsk_state_v1.bin.
// The golden file is produced by sim/bridge/packet.py. No libzmq and no F Prime.

#include "BasiliskStatePacket.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool near(double actual, double expected) {
    return actual == expected;
}

int fail(const std::string& message) {
    std::cerr << message << "\n";
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "tests/golden/bsk_state_v1.bin";
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return fail(std::string("could not open ") + path);
    }
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    Orbiedge::BasiliskState state{};
    const Orbiedge::PacketStatus status = Orbiedge::decodeBasiliskState(bytes.data(), bytes.size(), state);
    if (status != Orbiedge::PacketStatus::Ok) {
        return fail(std::string("decode failed: ") + Orbiedge::packetStatusText(status));
    }
    if (state.simTimeNs != 120000000000ull) {
        return fail("simTimeNs");
    }
    if (!near(state.qBn[0], 0.5) || !near(state.qBn[1], 0.5) || !near(state.qBn[2], 0.5) || !near(state.qBn[3], 0.5)) {
        return fail("qBn");
    }
    if (!near(state.omegaBnB[0], 0.125) || !near(state.omegaBnB[1], -0.25) || !near(state.omegaBnB[2], 0.5)) {
        return fail("omega");
    }
    if (!near(state.busVoltageDerived, 8.0) || !near(state.currentNetPowerW, -1.5) || !near(state.soc, 0.75)) {
        return fail("power");
    }
    if (!near(state.wheelSpeedRadS[0], 1.0) || !near(state.wheelSpeedRadS[1], 2.0) || !near(state.wheelSpeedRadS[2], 3.0)) {
        return fail("wheels");
    }
    if (!near(state.gyroAngVelRadS[0], 0.25) || !near(state.gyroAngVelRadS[1], 0.0) || !near(state.gyroAngVelRadS[2], -0.25)) {
        return fail("gyro");
    }
    if (!near(state.starTrackerQ[0], 1.0) || state.starTrackerQ[1] != 0.0 || state.starTrackerQ[2] != 0.0 ||
        state.starTrackerQ[3] != 0.0) {
        return fail("star tracker");
    }
    if (!near(state.css[0], 0.0) || !near(state.css[1], 1.0) || !near(state.css[2], 0.5) || !near(state.css[3], 0.25)) {
        return fail("css");
    }
    if (!near(state.panelEfficiency, 0.125) || !near(state.neutralDensityKgM3, 0.0)) {
        return fail("environment");
    }
    const std::uint16_t expectedFlags = Orbiedge::FAULT_RW3_STICTION | Orbiedge::FAULT_IMU_WALK;
    if (state.faultFlags != expectedFlags || state.sequence != 7u) {
        return fail("flags or sequence");
    }

    std::vector<std::uint8_t> broken = bytes;
    broken[16] = static_cast<std::uint8_t>(broken[16] ^ 0x01u);
    Orbiedge::BasiliskState ignored{};
    if (Orbiedge::decodeBasiliskState(broken.data(), broken.size(), ignored) != Orbiedge::PacketStatus::Crc) {
        return fail("corrupted packet was accepted");
    }
    if (Orbiedge::decodeBasiliskState(bytes.data(), bytes.size() - 1, ignored) != Orbiedge::PacketStatus::Size) {
        return fail("short packet was accepted");
    }
    std::cout << "packet unpack ok\n";
    return 0;
}
