#ifndef ORBIEDGE_TELEMETRYMONITOR_HPP
#define ORBIEDGE_TELEMETRYMONITOR_HPP

#include "Orbiedge/TelemetryMonitor/TelemetryMonitorComponentAc.hpp"

namespace Orbiedge {

// Subscribes to Basilisk state packets. START only opens the socket.
// schedIn polls one datagram with a zero timeout on this component's thread.
class TelemetryMonitor final : public TelemetryMonitorComponentBase {
  public:
    explicit TelemetryMonitor(const char* const compName);
    ~TelemetryMonitor() override;

  private:
    void START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Fw::CmdStringArg& endpoint) override;
    void STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void schedIn_handler(FwIndexType portNum, U32 context) override;

    void closeSocket();
    void pollOne();
    void rejectPacket(const char* reason);

    void* m_context;
    void* m_socket;
    bool m_running;
    U32 m_accepted;
};

}  // namespace Orbiedge

#endif
