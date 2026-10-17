// Basilisk state packet, version 1.
//
// 256-byte little-endian image. Offsets match sim/bridge/packet.py.
// This header does not include F Prime or ZeroMQ. The framework-free
// unpack test compiles it on its own.
//
// Quaternion order is Basilisk MRP2EP: index 0 is the scalar.
// busVoltageDerived is computed by the bridge from state of charge.
// It is not a SimpleBattery measurement.

#ifndef ORBIEDGE_BASILISK_STATE_PACKET_HPP
#define ORBIEDGE_BASILISK_STATE_PACKET_HPP

#include <cstdint>
#include <cstring>

namespace Orbiedge {

static constexpr std::uint32_t BSK_MAGIC = 0x42534B31u;
static constexpr std::uint16_t BSK_VERSION = 1u;
static constexpr std::size_t BSK_PACKET_SIZE = 256u;
static constexpr std::size_t BSK_PAYLOAD_SIZE = 252u;

static constexpr std::uint16_t FAULT_RW3_STICTION = 1u << 0;
static constexpr std::uint16_t FAULT_BROWNOUT = 1u << 1;
static constexpr std::uint16_t FAULT_IMU_WALK = 1u << 2;

enum class PacketStatus {
    Ok = 0,
    Size,
    Magic,
    Version,
    Crc,
    Endian
};

struct BasiliskState {
    std::uint64_t simTimeNs;
    double qBn[4];
    double omegaBnB[3];
    double busVoltageDerived;
    double currentNetPowerW;
    double soc;
    double wheelSpeedRadS[3];
    double gyroAngVelRadS[3];
    double starTrackerQ[4];
    double css[4];
    double panelEfficiency;
    double neutralDensityKgM3;
    std::uint16_t faultFlags;
    std::uint32_t sequence;
};

inline bool hostIsLittleEndian() {
    const std::uint16_t value = 1u;
    return *reinterpret_cast<const std::uint8_t*>(&value) == 1u;
}

inline std::uint32_t crc32(const std::uint8_t* data, std::size_t length) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t index = 0; index < length; ++index) {
        crc ^= static_cast<std::uint32_t>(data[index]);
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

inline std::uint16_t readU16(const std::uint8_t* data) {
    std::uint16_t value = 0;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

inline std::uint32_t readU32(const std::uint8_t* data) {
    std::uint32_t value = 0;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

inline std::uint64_t readU64(const std::uint8_t* data) {
    std::uint64_t value = 0;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

inline double readF64(const std::uint8_t* data) {
    double value = 0.0;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

inline void readF64Array(const std::uint8_t* data, double* out, int count) {
    for (int index = 0; index < count; ++index) {
        out[index] = readF64(data + static_cast<std::size_t>(index) * sizeof(double));
    }
}

inline PacketStatus decodeBasiliskState(const std::uint8_t* packet, std::size_t length, BasiliskState& out) {
    if (!hostIsLittleEndian()) {
        return PacketStatus::Endian;
    }
    if (packet == nullptr || length != BSK_PACKET_SIZE) {
        return PacketStatus::Size;
    }
    const std::uint32_t actualCrc = crc32(packet, BSK_PAYLOAD_SIZE);
    const std::uint32_t expectedCrc = readU32(packet + BSK_PAYLOAD_SIZE);
    if (actualCrc != expectedCrc) {
        return PacketStatus::Crc;
    }
    const std::uint32_t magic = readU32(packet + 0);
    if (magic != BSK_MAGIC) {
        return PacketStatus::Magic;
    }
    const std::uint16_t version = readU16(packet + 4);
    if (version != BSK_VERSION) {
        return PacketStatus::Version;
    }

    out.faultFlags = readU16(packet + 6);
    out.simTimeNs = readU64(packet + 8);
    readF64Array(packet + 16, out.qBn, 4);
    readF64Array(packet + 48, out.omegaBnB, 3);
    out.busVoltageDerived = readF64(packet + 72);
    out.currentNetPowerW = readF64(packet + 80);
    out.soc = readF64(packet + 88);
    readF64Array(packet + 96, out.wheelSpeedRadS, 3);
    readF64Array(packet + 120, out.gyroAngVelRadS, 3);
    readF64Array(packet + 144, out.starTrackerQ, 4);
    readF64Array(packet + 176, out.css, 4);
    out.panelEfficiency = readF64(packet + 208);
    out.neutralDensityKgM3 = readF64(packet + 216);
    out.sequence = readU32(packet + 224);
    return PacketStatus::Ok;
}

inline const char* packetStatusText(PacketStatus status) {
    switch (status) {
        case PacketStatus::Ok:
            return "ok";
        case PacketStatus::Size:
            return "size";
        case PacketStatus::Magic:
            return "magic";
        case PacketStatus::Version:
            return "version";
        case PacketStatus::Crc:
            return "crc";
        case PacketStatus::Endian:
            return "endian";
        default:
            return "unknown";
    }
}

}  // namespace Orbiedge

#endif
