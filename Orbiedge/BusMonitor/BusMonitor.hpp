#ifndef ORBIEDGE_BUSMONITOR_HPP
#define ORBIEDGE_BUSMONITOR_HPP

#include "Orbiedge/BusMonitor/BusMonitorComponentAc.hpp"

#include "Os/Mutex.hpp"

namespace Orbiedge {

// Demo predicate limits. These are not vehicle qualification limits.
class BusMonitor final : public BusMonitorComponentBase {
  public:
    explicit BusMonitor(const char* const compName);
    ~BusMonitor() override;

  private:
    static constexpr U32 HIST = 20;
    static constexpr F64 WHEEL_MAX_RAD_S = 650.0;
    static constexpr F64 WHEEL_WARN_FRAC = 0.9;
    static constexpr F64 SOC_WARN = 0.30;
    static constexpr F64 SOC_CRIT = 0.15;
    static constexpr F64 TEMP_WARN_C = 40.0;
    static constexpr F64 TEMP_CRIT_C = 60.0;
    static constexpr U64 ONE_SEC_NS = 1000000000ULL;
    static constexpr U64 TEMP_CRIT_NS = 5000000000ULL;
    static constexpr U64 TEMP_WARN_NS = 10000000000ULL;
    static constexpr U64 DUMP_RISE_NS = 30000000000ULL;

    struct Decision {
        Severity severity;
        Mode mode;
        U8 wheel;
        U8 eligible;
    };

    void powerIn_handler(FwIndexType portNum, U64 simTimeNs, F64 soc, F64 solarScale) override;
    void thermalIn_handler(FwIndexType portNum, U64 simTimeNs, F64 tempC, bool heaterOn) override;
    void adcsIn_handler(FwIndexType portNum, U64 simTimeNs, F64 wheel0, F64 wheel1, F64 wheel2, F64 wheel3) override;
    void commitIn_handler(FwIndexType portNum, U64 simTimeNs) override;
    void currentMode_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) override;
    void clearSticky_handler(FwIndexType portNum, U8 source) override;

    void updateSoc(U64 simTimeNs, F64 soc);
    void updateTemp(U64 simTimeNs, F64 tempC);
    void updateWheels(U64 simTimeNs, const F64 wheels[4]);
    Decision evaluate(U64 simTimeNs) const;
    void publish(U64 simTimeNs);
    static F64 magnitude(F64 value);
    F64 wheelStddev(U32 index) const;

    Os::Mutex m_mutex;
    bool m_havePower;
    bool m_haveThermal;
    bool m_haveAdcs;
    F64 m_soc;
    F64 m_solar;
    F64 m_temp;
    bool m_heaterOn;
    F64 m_wheels[4];
    F64 m_hist[4][HIST];
    U32 m_histCount;
    U32 m_histNext;
    F64 m_maxWheel;
    U32 m_wheelWarnSamples;
    U32 m_socWarnSamples;
    U32 m_socCritSamples;
    U64 m_lastSocNs;
    bool m_haveSocSample;
    bool m_tempCritTiming;
    U64 m_tempCritSince;
    bool m_tempWarnTiming;
    U64 m_tempWarnSince;
    bool m_dumpTiming;
    U64 m_dumpSince;
    F64 m_dumpSpeedStart;
    Mode m_currentMode;
    bool m_stickySafe;
    bool m_stickyIsolate;
    U8 m_isolateWheel;
    Severity m_sentSeverity;
    Mode m_sentMode;
    U8 m_sentWheel;
};

}  // namespace Orbiedge

#endif
