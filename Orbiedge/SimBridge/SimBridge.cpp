#include "Orbiedge/SimBridge/SimBridge.hpp"

#include "Orbiedge/SimBridge/JsonLine.hpp"

#include <cstring>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

const char* modeText(Orbiedge::Mode mode) {
    switch (mode) {
        case Orbiedge::Mode::ISOLATE_RW:
            return "ISOLATE_RW";
        case Orbiedge::Mode::MOMENTUM_DUMP:
            return "MOMENTUM_DUMP";
        case Orbiedge::Mode::LOAD_SHED:
            return "LOAD_SHED";
        case Orbiedge::Mode::SAFE:
            return "SAFE";
        case Orbiedge::Mode::NOMINAL:
        default:
            return "NOMINAL";
    }
}

}  // namespace

namespace Orbiedge {

SimBridge::SimBridge(const char* const compName)
    : SimBridgeComponentBase(compName),
      m_simTimeNs(0),
      m_soc(0.0),
      m_solarScale(0.0),
      m_tempC(0.0),
      m_heaterOn(false),
      m_haveSample(false),
      m_lossLatched(false),
      m_rxLen(0),
      m_txLen(0),
      m_txOff(0),
      m_sensorFd(-1),
      m_listenFd(-1),
      m_modeFd(-1),
      m_connectPending(false),
      m_lastRx(std::chrono::steady_clock::now()) {
    for (int wheel = 0; wheel < 4; ++wheel) {
        this->m_wheels[wheel] = 0.0;
    }
}

SimBridge::~SimBridge() {
    this->closeSockets();
}

void SimBridge::stampSimTime(U64 simTimeNs) {
    // No Svc.Time set-port call. See the class comment.
    this->m_simTimeNs = simTimeNs;
}

void SimBridge::schedIn_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;
    this->pollBasiliskSockets();
}

void SimBridge::modeIn_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) {
    (void)portNum;
    const char* name = modeText(mode);
    const int wrote = JsonLine::write_mode(this->m_tx, TX_CAPACITY, simTimeNs, name, wheelIndex);
    if (wrote < 0) {
        this->m_txLen = 0;
        this->m_txOff = 0;
        return;
    }
    this->m_txLen = wrote;
    this->m_txOff = 0;
    this->pollBasiliskSockets();
}

void SimBridge::emitSample(bool valid) {
    this->tlmWrite_SENSOR_VALID(valid);
    if (this->isConnected_PowerState_OutputPort(0)) {
        this->PowerState_out(0, this->m_simTimeNs, this->m_soc, this->m_solarScale, valid);
    }
    if (this->isConnected_ThermalState_OutputPort(0)) {
        this->ThermalState_out(0, this->m_simTimeNs, this->m_tempC, this->m_heaterOn, valid);
    }
    if (this->isConnected_AdcsState_OutputPort(0)) {
        this->AdcsState_out(0, this->m_simTimeNs, this->m_wheels[0], this->m_wheels[1], this->m_wheels[2],
                            this->m_wheels[3], valid);
    }
}

void SimBridge::noteSensorLoss() {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - this->m_lastRx).count();
    if (elapsed < static_cast<std::chrono::nanoseconds::rep>(SENSOR_LOSS_WALL_NS)) {
        return;
    }
    this->tlmWrite_SENSOR_VALID(false);
    if (this->m_lossLatched) {
        return;
    }
    this->m_lossLatched = true;
    if (this->m_haveSample) {
        this->emitSample(false);
    }
    this->log_WARNING_HI_SensorLoss(this->m_simTimeNs);
}

void SimBridge::closeSockets() {
#if defined(__linux__) || defined(__APPLE__)
    if (this->m_sensorFd >= 0) {
        ::close(this->m_sensorFd);
        this->m_sensorFd = -1;
    }
    if (this->m_modeFd >= 0) {
        ::close(this->m_modeFd);
        this->m_modeFd = -1;
    }
    if (this->m_listenFd >= 0) {
        ::close(this->m_listenFd);
        this->m_listenFd = -1;
    }
#endif
    this->m_connectPending = false;
}

void SimBridge::pollBasiliskSockets() {
    // TCP client to 127.0.0.1:5600 and mode writer bound to 127.0.0.1:5601.
    // Called only from schedIn and modeIn, both async on this component thread.
    // Non-blocking. A full rate-group queue drops the tick instead of waiting here.
#if defined(__linux__) || defined(__APPLE__)
    if (this->m_listenFd < 0) {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            this->log_WARNING_LO_LinkFailed();
        } else {
            const int yes = 1;
            ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
            const int flags = ::fcntl(fd, F_GETFL, 0);
            ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_port = htons(5601);
            if (::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1 ||
                ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 || ::listen(fd, 1) != 0) {
                ::close(fd);
                this->log_WARNING_LO_LinkFailed();
            } else {
                this->m_listenFd = fd;
            }
        }
    }
    if (this->m_sensorFd < 0) {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            this->log_WARNING_LO_LinkFailed();
        } else {
            const int flags = ::fcntl(fd, F_GETFL, 0);
            ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_port = htons(5600);
            if (::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1) {
                ::close(fd);
                this->log_WARNING_LO_LinkFailed();
            } else {
                const int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
                if (rc == 0) {
                    this->m_sensorFd = fd;
                    this->m_connectPending = false;
                } else if (errno == EINPROGRESS) {
                    this->m_sensorFd = fd;
                    this->m_connectPending = true;
                } else {
                    ::close(fd);
                    this->log_WARNING_LO_LinkFailed();
                }
            }
        }
    } else if (this->m_connectPending) {
        int so_error = 0;
        socklen_t length = sizeof(so_error);
        if (::getsockopt(this->m_sensorFd, SOL_SOCKET, SO_ERROR, &so_error, &length) != 0 || so_error != 0) {
            ::close(this->m_sensorFd);
            this->m_sensorFd = -1;
            this->m_connectPending = false;
        } else {
            this->m_connectPending = false;
        }
    }
    if (this->m_listenFd >= 0 && this->m_modeFd < 0) {
        sockaddr_in peer{};
        socklen_t length = sizeof(peer);
        const int accepted = ::accept(this->m_listenFd, reinterpret_cast<sockaddr*>(&peer), &length);
        if (accepted >= 0) {
            char text[INET_ADDRSTRLEN];
            const char* printed = ::inet_ntop(AF_INET, &peer.sin_addr, text, sizeof(text));
            if (printed == nullptr || std::strcmp(printed, "127.0.0.1") != 0) {
                ::close(accepted);
            } else {
                const int flags = ::fcntl(accepted, F_GETFL, 0);
                ::fcntl(accepted, F_SETFL, flags | O_NONBLOCK);
                this->m_modeFd = accepted;
            }
        }
    }
    if (this->m_sensorFd >= 0 && !this->m_connectPending) {
        for (int spins = 0; spins < 8; ++spins) {
            if (this->m_rxLen >= RX_CAPACITY - 1) {
                this->m_rxLen = 0;
                this->log_WARNING_LO_ParseFailed();
                break;
            }
            const ssize_t got = ::recv(this->m_sensorFd, this->m_rx + this->m_rxLen,
                                       static_cast<std::size_t>(RX_CAPACITY - 1 - this->m_rxLen), 0);
            if (got == 0) {
                ::close(this->m_sensorFd);
                this->m_sensorFd = -1;
                break;
            }
            if (got < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ::close(this->m_sensorFd);
                    this->m_sensorFd = -1;
                }
                break;
            }
            this->m_rxLen += static_cast<int>(got);
            this->m_rx[this->m_rxLen] = '\0';
            int consumed = 0;
            for (int index = 0; index < this->m_rxLen; ++index) {
                if (this->m_rx[index] != '\n') {
                    continue;
                }
                this->m_rx[index] = '\0';
                JsonLine parsed{};
                if (JsonLine::parse(this->m_rx + consumed, parsed) && parsed.kind == JsonLine::Kind::SENSOR) {
                    this->stampSimTime(parsed.sim_time_ns);
                    this->m_soc = parsed.soc;
                    this->m_solarScale = parsed.solar_scale;
                    this->m_tempC = parsed.temp_c;
                    this->m_heaterOn = parsed.heater_on;
                    for (int wheel = 0; wheel < 4; ++wheel) {
                        this->m_wheels[wheel] = parsed.wheel_speed_rad_s[wheel];
                    }
                    this->m_haveSample = true;
                    this->m_lossLatched = false;
                    this->m_lastRx = std::chrono::steady_clock::now();
                    this->emitSample(true);
                } else {
                    this->log_WARNING_LO_ParseFailed();
                }
                consumed = index + 1;
            }
            if (consumed > 0) {
                const int remain = this->m_rxLen - consumed;
                if (remain > 0) {
                    std::memmove(this->m_rx, this->m_rx + consumed, static_cast<std::size_t>(remain));
                }
                this->m_rxLen = remain;
            }
        }
    }
    if (this->m_modeFd >= 0 && this->m_txOff < this->m_txLen) {
#ifdef MSG_NOSIGNAL
        const int send_flags = MSG_NOSIGNAL;
#else
        const int send_flags = 0;
#endif
        const ssize_t sent = ::send(this->m_modeFd, this->m_tx + this->m_txOff,
                                    static_cast<std::size_t>(this->m_txLen - this->m_txOff), send_flags);
        if (sent > 0) {
            this->m_txOff += static_cast<int>(sent);
            if (this->m_txOff >= this->m_txLen) {
                this->m_txLen = 0;
                this->m_txOff = 0;
            }
        } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ::close(this->m_modeFd);
            this->m_modeFd = -1;
        }
    }
#else
    (void)this->m_sensorFd;
#endif
    this->noteSensorLoss();
}

}  // namespace Orbiedge
