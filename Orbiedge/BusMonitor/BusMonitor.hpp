#ifndef ORBIEDGE_BUSMONITOR_HPP
#define ORBIEDGE_BUSMONITOR_HPP

#include "Orbiedge/BusMonitor/BusMonitorComponentAc.hpp"
#include "Orbiedge/FdirRules/FdirRules.hpp"

#include <Os/Mutex.hpp>

namespace Orbiedge {

// Passive. Handlers only compare and count. No sockets.
class BusMonitor final : public BusMonitorComponentBase {
  public:
    explicit BusMonitor(const char* const compName);
    ~BusMonitor() override;

  private:
    void powerIn_handler(FwIndexType portNum, U64 simTimeNs, F64 soc, F64 solarScale, bool valid) override;
    void thermalIn_handler(FwIndexType portNum, U64 simTimeNs, F64 tempC, bool heaterOn, bool valid) override;
    void adcsIn_handler(FwIndexType portNum, U64 simTimeNs, F64 wheel0, F64 wheel1, F64 wheel2, F64 wheel3,
                        bool valid) override;
    void modeIn_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) override;
    void clearIn_handler(FwIndexType portNum, bool reset) override;

    void loadThresholds();
    void emit(const FdirRules::FaultRecord& record);

    Os::Mutex m_mutex;
    FdirRules m_rules;
    FdirRules::Mode m_mode;
    bool m_powerSeen;
    bool m_thermalSeen;
    U64 m_powerTime;
    U64 m_thermalTime;
    F64 m_soc;
    F64 m_solarScale;
    bool m_powerValid;
    F64 m_tempC;
    bool m_heaterOn;
    bool m_thermalValid;
};

}  // namespace Orbiedge

#endif
