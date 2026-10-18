#include "Orbiedge/SimBridge/SimBridge.hpp"

#include "Fw/Time/Time.hpp"
#include "Os/RawTime.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace {

const char* modeName(Orbiedge::Mode mode) {
    switch (mode) {
        case Orbiedge::Mode::NOMINAL:
            return "NOMINAL";
        case Orbiedge::Mode::ISOLATE_RW:
            return "ISOLATE_RW";
        case Orbiedge::Mode::MOMENTUM_DUMP:
            return "MOMENTUM_DUMP";
        case Orbiedge::Mode::LOAD_SHED:
            return "LOAD_SHED";
        case Orbiedge::Mode::SAFE:
            return "SAFE";
        default:
            return "NOMINAL";
    }
}

bool modeFromName(const char* text, U32 length, Orbiedge::Mode& mode) {
    struct Item {
        const char* name;
        Orbiedge::Mode mode;
    };
    const Item items[] = {
        {"NOMINAL", Orbiedge::Mode::NOMINAL},
        {"ISOLATE_RW", Orbiedge::Mode::ISOLATE_RW},
        {"MOMENTUM_DUMP", Orbiedge::Mode::MOMENTUM_DUMP},
        {"LOAD_SHED", Orbiedge::Mode::LOAD_SHED},
        {"SAFE", Orbiedge::Mode::SAFE},
    };
    for (U32 index = 0; index < 5U; index++) {
        const U32 nameLength = static_cast<U32>(std::strlen(items[index].name));
        if (nameLength == length && std::strncmp(text, items[index].name, length) == 0) {
            mode = items[index].mode;
            return true;
        }
    }
    return false;
}

const char* afterKey(const char* line, const char* key) {
    char pattern[64];
    const int wrote = std::snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    if (wrote <= 0 || static_cast<size_t>(wrote) >= sizeof(pattern)) {
        return nullptr;
    }
    const char* found = std::strstr(line, pattern);
    if (found == nullptr) {
        return nullptr;
    }
    found += std::strlen(pattern);
    while (*found == ' ' || *found == '\t') {
        found++;
    }
    if (*found != ':') {
        return nullptr;
    }
    found++;
    while (*found == ' ' || *found == '\t') {
        found++;
    }
    return found;
}

bool readF64(const char* line, const char* key, F64& out) {
    const char* text = afterKey(line, key);
    if (text == nullptr) {
        return false;
    }
    char* end = nullptr;
    const F64 value = std::strtod(text, &end);
    if (end == text || !std::isfinite(value)) {
        return false;
    }
    out = value;
    return true;
}

bool readU64(const char* line, const char* key, U64& out) {
    const char* text = afterKey(line, key);
    if (text == nullptr || *text == '"') {
        return false;
    }
    char* end = nullptr;
    const unsigned long long value = std::strtoull(text, &end, 10);
    if (end == text) {
        return false;
    }
    out = static_cast<U64>(value);
    return true;
}

bool readU32(const char* line, const char* key, U32& out) {
    U64 value = 0;
    if (!readU64(line, key, value) || value > 0xFFFFFFFFULL) {
        return false;
    }
    out = static_cast<U32>(value);
    return true;
}

int setNonblock(int fd) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return -1;
    }
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

}  // namespace

namespace Orbiedge {

SimBridge::SimBridge(const char* const compName)
    : SimBridgeComponentBase(compName),
      m_mutex(),
      m_started(false),
      m_sensorFd(-1),
      m_sensorState(0),
      m_modeListen(-1),
      m_uplinkListen(-1),
      m_modeClients{},
      m_uplinkClients{},
      m_sensorLine{},
      m_sensorLength(0),
      m_haveSample(false),
      m_lastSampleMs(0),
      m_sensorLost(false),
      m_haveMode(false),
      m_modeTime(0),
      m_mode(Mode::NOMINAL),
      m_wheel(0) {
    for (U32 index = 0; index < SimBridge::MODE_CLIENTS; index++) {
        this->m_modeClients[index].fd = -1;
        this->m_modeClients[index].length = 0;
    }
    for (U32 index = 0; index < SimBridge::UPLINK_CLIENTS; index++) {
        this->m_uplinkClients[index].fd = -1;
        this->m_uplinkClients[index].length = 0;
    }
}

SimBridge::~SimBridge() {
    this->closeFd(this->m_sensorFd);
    this->closeFd(this->m_modeListen);
    this->closeFd(this->m_uplinkListen);
    for (U32 index = 0; index < SimBridge::MODE_CLIENTS; index++) {
        this->closeFd(this->m_modeClients[index].fd);
    }
    for (U32 index = 0; index < SimBridge::UPLINK_CLIENTS; index++) {
        this->closeFd(this->m_uplinkClients[index].fd);
    }
}

void SimBridge::closeFd(int& fd) {
    if (fd >= 0) {
        (void)::close(fd);
        fd = -1;
    }
}

U64 SimBridge::wallMs() {
    timespec now = {};
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0 || now.tv_sec < 0 || now.tv_nsec < 0) {
        return 0;
    }
    return (static_cast<U64>(now.tv_sec) * 1000ULL) + static_cast<U64>(now.tv_nsec / 1000000L);
}

void SimBridge::bindListener(int port, int& fd) {
    this->closeFd(fd);
    const int opened = ::socket(AF_INET, SOCK_STREAM, 0);
    if (opened < 0) {
        this->log_WARNING_HI_BindFailed(static_cast<U16>(port));
        return;
    }
    const int enable = 1;
    (void)::setsockopt(opened, SOL_SOCKET, SO_REUSEADDR, &enable, static_cast<socklen_t>(sizeof(enable)));
    if (setNonblock(opened) != 0) {
        (void)::close(opened);
        this->log_WARNING_HI_BindFailed(static_cast<U16>(port));
        return;
    }
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1) {
        (void)::close(opened);
        this->log_WARNING_HI_BindFailed(static_cast<U16>(port));
        return;
    }
    if (::bind(opened, reinterpret_cast<sockaddr*>(&address), static_cast<socklen_t>(sizeof(address))) != 0 ||
        ::listen(opened, 2) != 0) {
        (void)::close(opened);
        this->log_WARNING_HI_BindFailed(static_cast<U16>(port));
        return;
    }
    fd = opened;
    this->log_ACTIVITY_HI_Listening(static_cast<U16>(port));
}

void SimBridge::startSockets() {
    Os::ScopeLock lock(this->m_mutex);
    if (this->m_started) {
        return;
    }
    this->m_started = true;
    this->bindListener(SimBridge::MODE_PORT, this->m_modeListen);
    this->bindListener(SimBridge::UPLINK_PORT, this->m_uplinkListen);
}

void SimBridge::connectSensor() {
    if (this->m_sensorState == 2 && this->m_sensorFd >= 0) {
        return;
    }
    if (this->m_sensorState == 1 && this->m_sensorFd >= 0) {
        pollfd waiting = {};
        waiting.fd = this->m_sensorFd;
        waiting.events = POLLOUT;
        if (::poll(&waiting, static_cast<nfds_t>(1), 0) <= 0) {
            return;
        }
        int error = 0;
        socklen_t length = static_cast<socklen_t>(sizeof(error));
        if (::getsockopt(this->m_sensorFd, SOL_SOCKET, SO_ERROR, &error, &length) != 0 || error != 0) {
            this->closeFd(this->m_sensorFd);
            this->m_sensorState = 0;
        } else {
            this->m_sensorState = 2;
        }
        return;
    }
    const int opened = ::socket(AF_INET, SOCK_STREAM, 0);
    if (opened < 0) {
        return;
    }
    if (setNonblock(opened) != 0) {
        (void)::close(opened);
        return;
    }
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(SimBridge::SENSOR_PORT));
    if (::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1) {
        (void)::close(opened);
        return;
    }
    const int result =
        ::connect(opened, reinterpret_cast<sockaddr*>(&address), static_cast<socklen_t>(sizeof(address)));
    if (result == 0) {
        this->m_sensorFd = opened;
        this->m_sensorState = 2;
        return;
    }
    if (errno == EINPROGRESS) {
        this->m_sensorFd = opened;
        this->m_sensorState = 1;
        return;
    }
    (void)::close(opened);
    this->m_sensorState = 0;
}

void SimBridge::rememberMode(U64 simTimeNs, Mode mode, U8 wheelIndex) {
    this->m_haveMode = true;
    this->m_modeTime = simTimeNs;
    this->m_mode = mode;
    this->m_wheel = wheelIndex;
}

void SimBridge::writeMode(U64 simTimeNs, Mode mode, U8 wheelIndex) {
    char line[160];
    const int wrote = std::snprintf(line, sizeof(line), "{\"sim_time_ns\":%llu,\"mode\":\"%s\",\"wheel_index\":%u}\n",
                                    static_cast<unsigned long long>(simTimeNs), modeName(mode),
                                    static_cast<unsigned>(wheelIndex));
    if (wrote <= 0 || static_cast<size_t>(wrote) >= sizeof(line)) {
        return;
    }
    const U32 size = static_cast<U32>(wrote);
    for (U32 index = 0; index < SimBridge::MODE_CLIENTS; index++) {
        int& fd = this->m_modeClients[index].fd;
        if (fd < 0) {
            continue;
        }
        U32 sent = 0;
        while (sent < size) {
            const ssize_t result = ::send(fd, line + sent, static_cast<size_t>(size - sent), MSG_NOSIGNAL);
            if (result < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }
                this->closeFd(fd);
                break;
            }
            sent = sent + static_cast<U32>(result);
        }
    }
}

void SimBridge::acceptClients(int listenFd, Client* clients, U32 capacity, bool uplink) {
    if (listenFd < 0) {
        return;
    }
    while (true) {
        const int opened = ::accept(listenFd, nullptr, nullptr);
        if (opened < 0) {
            return;
        }
        if (setNonblock(opened) != 0) {
            (void)::close(opened);
            continue;
        }
        U32 slot = capacity;
        for (U32 index = 0; index < capacity; index++) {
            if (clients[index].fd < 0) {
                slot = index;
                break;
            }
        }
        if (slot == capacity) {
            (void)::close(opened);
            return;
        }
        clients[slot].fd = opened;
        clients[slot].length = 0;
        if (!uplink && this->m_haveMode) {
            this->writeMode(this->m_modeTime, this->m_mode, this->m_wheel);
        }
    }
}

bool SimBridge::parseSensor(const char* line, U64& simTimeNs, F64& soc, F64& solar, F64 wheels[4], F64& tempC,
                            bool& heaterOn) {
    if (!readU64(line, "sim_time_ns", simTimeNs) || !readF64(line, "soc", soc) || !readF64(line, "solar_scale", solar) ||
        !readF64(line, "temp_c", tempC)) {
        return false;
    }
    const char* wheelsText = afterKey(line, "wheel_speed_rad_s");
    if (wheelsText == nullptr || *wheelsText != '[') {
        return false;
    }
    const char* cursor = wheelsText + 1;
    for (U32 index = 0; index < 4U; index++) {
        char* end = nullptr;
        const F64 value = std::strtod(cursor, &end);
        if (end == cursor || !std::isfinite(value)) {
            return false;
        }
        wheels[index] = value;
        cursor = end;
        while (*cursor == ' ' || *cursor == ',') {
            cursor++;
        }
    }
    const char* heater = afterKey(line, "heater_on");
    if (heater == nullptr) {
        return false;
    }
    if (std::strncmp(heater, "true", 4) == 0) {
        heaterOn = true;
        return true;
    }
    if (std::strncmp(heater, "false", 5) == 0) {
        heaterOn = false;
        return true;
    }
    return false;
}

bool SimBridge::parseProposal(const char* line, U32& opcode, Mode& action, U8& wheelIndex, F32& confidence,
                              U64& simTimeNs) {
    U32 wheel = 0;
    F64 score = 0.0;
    if (!readU32(line, "opcode", opcode) || !readU64(line, "sim_time_ns", simTimeNs) ||
        !readU32(line, "wheel_index", wheel) || !readF64(line, "confidence", score)) {
        return false;
    }
    const char* name = afterKey(line, "action");
    if (name == nullptr || *name != '"') {
        return false;
    }
    const char* end = std::strchr(name + 1, '"');
    if (end == nullptr) {
        return false;
    }
    const U32 length = static_cast<U32>(end - (name + 1));
    if (wheel > 3U || !modeFromName(name + 1, length, action)) {
        return false;
    }
    wheelIndex = static_cast<U8>(wheel);
    confidence = static_cast<F32>(score);
    return true;
}

void SimBridge::readUplink(Client& client, ParsedProposal* out, U32 capacity, U32& count) {
    if (client.fd < 0) {
        return;
    }
    while (client.length + 1U < SimBridge::LINE_MAX) {
        const ssize_t result = ::recv(client.fd, client.line + client.length,
                                      static_cast<size_t>(SimBridge::LINE_MAX - 1U - client.length), 0);
        if (result == 0) {
            this->closeFd(client.fd);
            client.length = 0;
            return;
        }
        if (result < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            this->closeFd(client.fd);
            client.length = 0;
            return;
        }
        client.length = client.length + static_cast<U32>(result);
        client.line[client.length] = '\0';
    }
    U32 start = 0;
    for (U32 index = 0; index < client.length; index++) {
        if (client.line[index] != '\n') {
            continue;
        }
        client.line[index] = '\0';
        ParsedProposal proposal = {};
        if (count < capacity &&
            SimBridge::parseProposal(client.line + start, proposal.opcode, proposal.action, proposal.wheelIndex,
                                     proposal.confidence, proposal.simTimeNs)) {
            out[count] = proposal;
            count = count + 1U;
        }
        start = index + 1U;
    }
    if (start > 0U) {
        const U32 remain = client.length - start;
        std::memmove(client.line, client.line + start, remain);
        client.length = remain;
        client.line[client.length] = '\0';
    } else if (client.length + 1U >= SimBridge::LINE_MAX) {
        client.length = 0;
    }
}

void SimBridge::readSensor(ParsedSensor* out, U32 capacity, U32& count) {
    if (this->m_sensorState != 2 || this->m_sensorFd < 0) {
        return;
    }
    while (this->m_sensorLength + 1U < SimBridge::LINE_MAX) {
        const ssize_t result = ::recv(this->m_sensorFd, this->m_sensorLine + this->m_sensorLength,
                                      static_cast<size_t>(SimBridge::LINE_MAX - 1U - this->m_sensorLength), 0);
        if (result == 0) {
            this->closeFd(this->m_sensorFd);
            this->m_sensorState = 0;
            this->m_sensorLength = 0;
            return;
        }
        if (result < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            this->closeFd(this->m_sensorFd);
            this->m_sensorState = 0;
            this->m_sensorLength = 0;
            return;
        }
        this->m_sensorLength = this->m_sensorLength + static_cast<U32>(result);
        this->m_sensorLine[this->m_sensorLength] = '\0';
    }
    U32 start = 0;
    for (U32 index = 0; index < this->m_sensorLength; index++) {
        if (this->m_sensorLine[index] != '\n') {
            continue;
        }
        this->m_sensorLine[index] = '\0';
        ParsedSensor sample = {};
        if (count < capacity && SimBridge::parseSensor(this->m_sensorLine + start, sample.simTimeNs, sample.soc,
                                                       sample.solar, sample.wheels, sample.tempC, sample.heaterOn)) {
            out[count] = sample;
            count = count + 1U;
            this->m_haveSample = true;
            this->m_lastSampleMs = SimBridge::wallMs();
            this->m_sensorLost = false;
        }
        start = index + 1U;
    }
    if (start > 0U) {
        const U32 remain = this->m_sensorLength - start;
        std::memmove(this->m_sensorLine, this->m_sensorLine + start, remain);
        this->m_sensorLength = remain;
        this->m_sensorLine[this->m_sensorLength] = '\0';
    } else if (this->m_sensorLength + 1U >= SimBridge::LINE_MAX) {
        this->m_sensorLength = 0;
    }
}

void SimBridge::modeIn_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->rememberMode(simTimeNs, mode, wheelIndex);
    this->writeMode(simTimeNs, mode, wheelIndex);
}

void SimBridge::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;
    if (!this->m_started) {
        this->startSockets();
    }
    ParsedSensor samples[8] = {};
    ParsedProposal proposals[8] = {};
    U32 sampleCount = 0;
    U32 proposalCount = 0;
    bool lostEdge = false;
    bool invalid = false;
    {
        Os::ScopeLock lock(this->m_mutex);
        this->connectSensor();
        this->acceptClients(this->m_modeListen, this->m_modeClients, SimBridge::MODE_CLIENTS, false);
        this->acceptClients(this->m_uplinkListen, this->m_uplinkClients, SimBridge::UPLINK_CLIENTS, true);
        this->readSensor(samples, 8U, sampleCount);
        for (U32 index = 0; index < SimBridge::UPLINK_CLIENTS; index++) {
            this->readUplink(this->m_uplinkClients[index], proposals, 8U, proposalCount);
        }
        if (this->m_haveSample) {
            const U64 now = SimBridge::wallMs();
            if (now >= this->m_lastSampleMs && (now - this->m_lastSampleMs) > SimBridge::SENSOR_LOSS_MS) {
                invalid = true;
                if (!this->m_sensorLost) {
                    this->m_sensorLost = true;
                    lostEdge = true;
                }
            }
        } else {
            invalid = true;
        }
    }
    for (U32 index = 0; index < sampleCount; index++) {
        const ParsedSensor& sample = samples[index];
        const U32 seconds = static_cast<U32>(sample.simTimeNs / 1000000000ULL);
        const U32 microseconds = static_cast<U32>((sample.simTimeNs % 1000000000ULL) / 1000ULL);
        Fw::Time flightTime(::TimeBase::TB_WORKSTATION_TIME, seconds, microseconds);
        Os::RawTime raw;
        (void)raw.now();
        this->timeEpochSet_out(0, flightTime, raw);
        this->filterSetTime_out(0, sample.simTimeNs);
        this->sensorValidOut_out(0, true);
        this->powerOut_out(0, sample.simTimeNs, sample.soc, sample.solar);
        this->thermalOut_out(0, sample.simTimeNs, sample.tempC, sample.heaterOn);
        this->adcsOut_out(0, sample.simTimeNs, sample.wheels[0], sample.wheels[1], sample.wheels[2], sample.wheels[3]);
        this->commitOut_out(0, sample.simTimeNs);
    }
    for (U32 index = 0; index < proposalCount; index++) {
        const ParsedProposal& proposal = proposals[index];
        this->proposalOut_out(0, proposal.opcode, proposal.action, proposal.wheelIndex, proposal.confidence,
                              proposal.simTimeNs);
    }
    if (lostEdge) {
        this->log_WARNING_HI_SensorLost();
    }
    if (invalid) {
        this->sensorValidOut_out(0, false);
    }
}

}  // namespace Orbiedge
