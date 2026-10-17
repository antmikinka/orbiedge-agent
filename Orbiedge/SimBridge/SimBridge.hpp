#ifndef ORBIEDGE_SIMBRIDGE_HPP
#define ORBIEDGE_SIMBRIDGE_HPP

#include "Orbiedge/SimBridge/SimBridgeComponentAc.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace Orbiedge {

// Active. Blocking sockets stay off the rate group.
// pollBasiliskSockets() is the only method that touches 127.0.0.1:5600 and :5601.
class SimBridge final : public SimBridgeComponentBase {
  public:
    explicit SimBridge(const char* const compName);
    ~SimBridge() override;

    // Typed sim-time hook. Svc.Time has no set port in this checkout
    // (Svc/Interfaces/Time.fpp is only timeGetPort). Do not call a guessed port.
    // The topology connects the real time-set port later; this stores Basilisk time.
    void stampSimTime(U64 simTimeNs);

  private:
    static constexpr U64 SENSOR_LOSS_WALL_NS = 500000000ULL;
    static constexpr int RX_CAPACITY = 2048;
    static constexpr int TX_CAPACITY = 256;

    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void modeIn_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) override;

    void pollBasiliskSockets();
    void noteSensorLoss();
    void emitSample(bool valid);
    void closeSockets();

    U64 m_simTimeNs;
    double m_soc;
    double m_solarScale;
    double m_wheels[4];
    double m_tempC;
    bool m_heaterOn;
    bool m_haveSample;
    bool m_lossLatched;
    char m_rx[RX_CAPACITY];
    int m_rxLen;
    char m_tx[TX_CAPACITY];
    int m_txLen;
    int m_txOff;
    int m_sensorFd;
    int m_listenFd;
    int m_modeFd;
    bool m_connectPending;
    std::chrono::steady_clock::time_point m_lastRx;
};

}  // namespace Orbiedge

#endif
