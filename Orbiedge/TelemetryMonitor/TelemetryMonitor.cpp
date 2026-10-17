#include "Orbiedge/TelemetryMonitor/TelemetryMonitor.hpp"

#include "Orbiedge/TelemetryMonitor/BasiliskStatePacket.hpp"

#include <Fw/Types/String.hpp>

#include <zmq.h>

#include <cerrno>
#include <cstring>

namespace {

constexpr int TOPIC_LENGTH = 9;
const char TOPIC[] = "bsk.state";

}  // namespace

namespace Orbiedge {

TelemetryMonitor::TelemetryMonitor(const char* const compName)
    : TelemetryMonitorComponentBase(compName),
      m_context(nullptr),
      m_socket(nullptr),
      m_running(false),
      m_accepted(0) {}

TelemetryMonitor::~TelemetryMonitor() {
    this->closeSocket();
}

void TelemetryMonitor::closeSocket() {
    if (this->m_socket != nullptr) {
        zmq_close(this->m_socket);
        this->m_socket = nullptr;
    }
    if (this->m_context != nullptr) {
        zmq_ctx_term(this->m_context);
        this->m_context = nullptr;
    }
    this->m_running = false;
}

void TelemetryMonitor::rejectPacket(const char* reason) {
    Fw::LogStringArg reasonArg(reason);
    this->log_WARNING_HI_PacketRejected(reasonArg);
}

void TelemetryMonitor::START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Fw::CmdStringArg& endpoint) {
    if (endpoint.length() == 0) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    this->closeSocket();
    this->m_context = zmq_ctx_new();
    if (this->m_context == nullptr) {
        this->rejectPacket("zmq context");
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    this->m_socket = zmq_socket(this->m_context, ZMQ_SUB);
    if (this->m_socket == nullptr) {
        this->closeSocket();
        this->rejectPacket("zmq socket");
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    const int timeoutMs = 0;
    const int lingerMs = 0;
    if ((zmq_setsockopt(this->m_socket, ZMQ_SUBSCRIBE, TOPIC, TOPIC_LENGTH) != 0) ||
        (zmq_setsockopt(this->m_socket, ZMQ_RCVTIMEO, &timeoutMs, sizeof(timeoutMs)) != 0) ||
        (zmq_setsockopt(this->m_socket, ZMQ_LINGER, &lingerMs, sizeof(lingerMs)) != 0) ||
        (zmq_connect(this->m_socket, endpoint.toChar()) != 0)) {
        this->closeSocket();
        this->rejectPacket("zmq connect");
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->m_running = true;
    Fw::LogStringArg endpointArg(endpoint.toChar());
    this->log_ACTIVITY_HI_Started(endpointArg);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void TelemetryMonitor::STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->closeSocket();
    this->log_ACTIVITY_HI_Stopped();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void TelemetryMonitor::schedIn_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;
    if (!this->m_running || (this->m_socket == nullptr)) {
        return;
    }
    this->pollOne();
}

void TelemetryMonitor::pollOne() {
    zmq_msg_t topic;
    zmq_msg_init(&topic);
    const int topicRc = zmq_msg_recv(&topic, this->m_socket, ZMQ_DONTWAIT);
    if (topicRc < 0) {
        zmq_msg_close(&topic);
        if (errno == EAGAIN) {
            return;
        }
        this->rejectPacket("zmq recv");
        return;
    }

    int more = 0;
    std::size_t moreSize = sizeof(more);
    zmq_getsockopt(this->m_socket, ZMQ_RCVMORE, &more, &moreSize);
    const bool topicOk =
        (static_cast<std::size_t>(topicRc) == TOPIC_LENGTH) && (std::memcmp(zmq_msg_data(&topic), TOPIC, TOPIC_LENGTH) == 0);
    zmq_msg_close(&topic);
    if (more == 0) {
        this->rejectPacket("missing packet frame");
        return;
    }

    zmq_msg_t body;
    zmq_msg_init(&body);
    const int bodyRc = zmq_msg_recv(&body, this->m_socket, ZMQ_DONTWAIT);
    if (bodyRc < 0) {
        zmq_msg_close(&body);
        this->rejectPacket("truncated packet");
        return;
    }

    // Drop any unexpected extra frames so the next poll starts on a topic.
    more = 0;
    zmq_getsockopt(this->m_socket, ZMQ_RCVMORE, &more, &moreSize);
    while (more != 0) {
        zmq_msg_t extra;
        zmq_msg_init(&extra);
        if (zmq_msg_recv(&extra, this->m_socket, ZMQ_DONTWAIT) < 0) {
            zmq_msg_close(&extra);
            break;
        }
        more = 0;
        zmq_getsockopt(this->m_socket, ZMQ_RCVMORE, &more, &moreSize);
        zmq_msg_close(&extra);
    }

    if (!topicOk) {
        zmq_msg_close(&body);
        this->rejectPacket("topic");
        return;
    }

    BasiliskState state{};
    const PacketStatus status = decodeBasiliskState(
        static_cast<const std::uint8_t*>(zmq_msg_data(&body)), static_cast<std::size_t>(bodyRc), state);
    zmq_msg_close(&body);
    if (status != PacketStatus::Ok) {
        this->rejectPacket(packetStatusText(status));
        return;
    }

    this->m_accepted += 1;
    this->tlmWrite_SimTimeNs(state.simTimeNs);
    F64x4 attitude;
    F64x3 rate;
    F64x3 wheels;
    for (int index = 0; index < 4; ++index) {
        attitude[index] = state.qBn[index];
    }
    for (int index = 0; index < 3; ++index) {
        rate[index] = state.omegaBnB[index];
        wheels[index] = state.wheelSpeedRadS[index];
    }
    this->tlmWrite_q_BN(attitude);
    this->tlmWrite_omega_BN_B(rate);
    this->tlmWrite_BusVoltage(state.busVoltageDerived);
    this->tlmWrite_WheelSpeeds(wheels);
    this->tlmWrite_FaultFlags(state.faultFlags);
    this->tlmWrite_PacketsAccepted(this->m_accepted);
    this->log_ACTIVITY_LO_PacketAccepted(state.simTimeNs, state.faultFlags);

    if (this->isConnected_stateOut_OutputPort(0)) {
        this->stateOut_out(0, state.simTimeNs, state.qBn[0], state.qBn[1], state.qBn[2], state.qBn[3],
                           state.omegaBnB[0], state.omegaBnB[1], state.omegaBnB[2], state.busVoltageDerived,
                           state.wheelSpeedRadS[0], state.wheelSpeedRadS[1], state.wheelSpeedRadS[2], state.faultFlags);
    }
}

}  // namespace Orbiedge
