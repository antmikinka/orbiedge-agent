#include "Orbiedge/BusMonitor/BusMonitor.hpp"

#include "Orbiedge/FdirTypes/FdirTypes.hpp"

#include <cmath>

namespace Orbiedge {

BusMonitor::BusMonitor(const char* const compName)
    : BusMonitorComponentBase(compName),
      m_mutex(),
      m_havePower(false),
      m_haveThermal(false),
      m_haveAdcs(false),
      m_soc(1.0),
      m_solar(1.0),
      m_temp(0.0),
      m_heaterOn(false),
      m_wheels{0.0, 0.0, 0.0, 0.0},
      m_hist{},
      m_histCount(0),
      m_histNext(0),
      m_maxWheel(0.0),
      m_wheelWarnSamples(0),
      m_socWarnSamples(0),
      m_socCritSamples(0),
      m_lastSocNs(0),
      m_haveSocSample(false),
      m_tempCritTiming(false),
      m_tempCritSince(0),
      m_tempWarnTiming(false),
      m_tempWarnSince(0),
      m_dumpTiming(false),
      m_dumpSince(0),
      m_dumpSpeedStart(0.0),
      m_currentMode(Mode::NOMINAL),
      m_stickySafe(false),
      m_stickyIsolate(false),
      m_isolateWheel(0),
      m_sentSeverity(Severity::NONE),
      m_sentMode(Mode::NOMINAL),
      m_sentWheel(0) {
    (void)fdirTypesLinkAnchor();
}

BusMonitor::~BusMonitor() {}

F64 BusMonitor::magnitude(F64 value) {
    return (value < 0.0) ? -value : value;
}

void BusMonitor::powerIn_handler(FwIndexType portNum, U64 simTimeNs, F64 soc, F64 solarScale) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->m_havePower = true;
    this->m_solar = solarScale;
    this->updateSoc(simTimeNs, soc);
}

void BusMonitor::thermalIn_handler(FwIndexType portNum, U64 simTimeNs, F64 tempC, bool heaterOn) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->m_haveThermal = true;
    this->m_heaterOn = heaterOn;
    this->updateTemp(simTimeNs, tempC);
}

void BusMonitor::adcsIn_handler(
    FwIndexType portNum, U64 simTimeNs, F64 wheel0, F64 wheel1, F64 wheel2, F64 wheel3) {
    (void)portNum;
    const F64 wheels[4] = {wheel0, wheel1, wheel2, wheel3};
    Os::ScopeLock lock(this->m_mutex);
    this->m_haveAdcs = true;
    this->updateWheels(simTimeNs, wheels);
}

void BusMonitor::commitIn_handler(FwIndexType portNum, U64 simTimeNs) {
    (void)portNum;
    this->publish(simTimeNs);
}

void BusMonitor::currentMode_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) {
    (void)portNum;
    (void)simTimeNs;
    (void)wheelIndex;
    Os::ScopeLock lock(this->m_mutex);
    this->m_currentMode = mode;
}

void BusMonitor::clearSticky_handler(FwIndexType portNum, U8 source) {
    (void)portNum;
    (void)source;
    U64 simTime = 0;
    {
        Os::ScopeLock lock(this->m_mutex);
        this->m_stickySafe = false;
        this->m_stickyIsolate = false;
        simTime = this->m_lastSocNs;
    }
    this->publish(simTime);
}

void BusMonitor::updateSoc(U64 simTimeNs, F64 soc) {
    this->m_soc = soc;
    if (this->m_haveSocSample && simTimeNs < this->m_lastSocNs + BusMonitor::ONE_SEC_NS) {
        return;
    }
    this->m_haveSocSample = true;
    this->m_lastSocNs = simTimeNs;
    if (soc < BusMonitor::SOC_CRIT) {
        this->m_socCritSamples = this->m_socCritSamples + 1U;
        this->m_socWarnSamples = this->m_socWarnSamples + 1U;
        return;
    }
    this->m_socCritSamples = 0;
    if (soc < BusMonitor::SOC_WARN) {
        this->m_socWarnSamples = this->m_socWarnSamples + 1U;
        return;
    }
    this->m_socWarnSamples = 0;
}

void BusMonitor::updateTemp(U64 simTimeNs, F64 tempC) {
    this->m_temp = tempC;
    if (tempC > BusMonitor::TEMP_CRIT_C) {
        if (!this->m_tempCritTiming) {
            this->m_tempCritTiming = true;
            this->m_tempCritSince = simTimeNs;
        }
    } else {
        this->m_tempCritTiming = false;
    }
    if (tempC > BusMonitor::TEMP_WARN_C && tempC <= BusMonitor::TEMP_CRIT_C) {
        if (!this->m_tempWarnTiming) {
            this->m_tempWarnTiming = true;
            this->m_tempWarnSince = simTimeNs;
        }
    } else {
        this->m_tempWarnTiming = false;
    }
}

void BusMonitor::updateWheels(U64 simTimeNs, const F64 wheels[4]) {
    F64 maxWheel = 0.0;
    for (U32 index = 0; index < 4U; index++) {
        this->m_wheels[index] = wheels[index];
        this->m_hist[index][this->m_histNext] = wheels[index];
        const F64 speed = BusMonitor::magnitude(wheels[index]);
        if (speed > maxWheel) {
            maxWheel = speed;
        }
    }
    this->m_histNext = (this->m_histNext + 1U) % BusMonitor::HIST;
    if (this->m_histCount < BusMonitor::HIST) {
        this->m_histCount = this->m_histCount + 1U;
    }
    this->m_maxWheel = maxWheel;
    if (maxWheel > BusMonitor::WHEEL_WARN_FRAC * BusMonitor::WHEEL_MAX_RAD_S) {
        this->m_wheelWarnSamples = this->m_wheelWarnSamples + 1U;
    } else {
        this->m_wheelWarnSamples = 0;
    }
    if (this->m_currentMode == Mode::MOMENTUM_DUMP) {
        if (!this->m_dumpTiming) {
            this->m_dumpTiming = true;
            this->m_dumpSince = simTimeNs;
            this->m_dumpSpeedStart = maxWheel;
        }
    } else {
        this->m_dumpTiming = false;
    }
}

F64 BusMonitor::wheelStddev(U32 index) const {
    if (this->m_histCount == 0U) {
        return 0.0;
    }
    F64 sum = 0.0;
    for (U32 sample = 0; sample < this->m_histCount; sample++) {
        sum = sum + this->m_hist[index][sample];
    }
    const F64 mean = sum / static_cast<F64>(this->m_histCount);
    F64 square = 0.0;
    for (U32 sample = 0; sample < this->m_histCount; sample++) {
        const F64 delta = this->m_hist[index][sample] - mean;
        square = square + (delta * delta);
    }
    return std::sqrt(square / static_cast<F64>(this->m_histCount));
}

BusMonitor::Decision BusMonitor::evaluate(U64 simTimeNs) const {
    Decision decision = {Severity::NONE, Mode::NOMINAL, 0, 0};
    if (this->m_stickySafe) {
        decision.severity = Severity::CRITICAL;
        decision.mode = Mode::SAFE;
        decision.eligible = 1;
        return decision;
    }
    if (this->m_havePower && this->m_socCritSamples >= 5U) {
        decision.severity = Severity::CRITICAL;
        decision.mode = Mode::SAFE;
        decision.eligible = 1;
        return decision;
    }
    if (this->m_haveThermal && this->m_tempCritTiming && simTimeNs >= this->m_tempCritSince &&
        (simTimeNs - this->m_tempCritSince) >= BusMonitor::TEMP_CRIT_NS) {
        decision.severity = Severity::CRITICAL;
        decision.mode = Mode::SAFE;
        decision.eligible = 1;
        return decision;
    }
    if (this->m_haveAdcs && this->m_dumpTiming && this->m_currentMode == Mode::MOMENTUM_DUMP &&
        simTimeNs >= this->m_dumpSince && (simTimeNs - this->m_dumpSince) >= BusMonitor::DUMP_RISE_NS &&
        this->m_maxWheel > this->m_dumpSpeedStart) {
        decision.severity = Severity::CRITICAL;
        decision.mode = Mode::SAFE;
        decision.eligible = 1;
        return decision;
    }
    if (this->m_stickyIsolate) {
        decision.severity = Severity::WARNING;
        decision.mode = Mode::ISOLATE_RW;
        decision.wheel = this->m_isolateWheel;
        decision.eligible = 2;
        return decision;
    }
    if (this->m_havePower && this->m_socWarnSamples >= 5U) {
        decision.severity = Severity::WARNING;
        decision.mode = Mode::LOAD_SHED;
        decision.eligible = 1;
        return decision;
    }
    if (this->m_haveAdcs && this->m_wheelWarnSamples >= 10U) {
        decision.severity = Severity::WARNING;
        decision.mode = Mode::MOMENTUM_DUMP;
        decision.eligible = 2;
        return decision;
    }
    if (this->m_haveAdcs && this->m_histCount >= BusMonitor::HIST) {
        U32 highIndex = 4U;
        U32 highCount = 0;
        bool quiet = true;
        for (U32 index = 0; index < 4U; index++) {
            const F64 spread = this->wheelStddev(index);
            if (spread > 0.05) {
                highCount = highCount + 1U;
                highIndex = index;
            } else if (!(spread < 0.01)) {
                quiet = false;
            }
        }
        if (highCount == 1U && quiet && highIndex < 4U) {
            decision.severity = Severity::WARNING;
            decision.mode = Mode::ISOLATE_RW;
            decision.wheel = static_cast<U8>(highIndex);
            decision.eligible = 2;
            return decision;
        }
    }
    return decision;
}

void BusMonitor::publish(U64 simTimeNs) {
    Decision decision;
    F64 temp = 0.0;
    F64 soc = 0.0;
    F64 maxWheel = 0.0;
    bool changed = false;
    {
        Os::ScopeLock lock(this->m_mutex);
        decision = this->evaluate(simTimeNs);
        if (decision.severity == Severity::CRITICAL && decision.mode == Mode::SAFE) {
            this->m_stickySafe = true;
        }
        if (decision.mode == Mode::ISOLATE_RW && decision.severity == Severity::WARNING) {
            this->m_stickyIsolate = true;
            this->m_isolateWheel = decision.wheel;
        }
        temp = this->m_temp;
        soc = this->m_soc;
        maxWheel = this->m_maxWheel;
        changed = decision.severity != this->m_sentSeverity || decision.mode != this->m_sentMode ||
                  decision.wheel != this->m_sentWheel;
        this->m_sentSeverity = decision.severity;
        this->m_sentMode = decision.mode;
        this->m_sentWheel = decision.wheel;
    }
    if (changed) {
        this->log_ACTIVITY_HI_Predicate(decision.severity, decision.mode, decision.wheel);
    }
    this->tlmWrite_Severity(decision.severity);
    this->tlmWrite_MaxWheel(maxWheel);
    this->faultOut_out(0, simTimeNs, decision.severity, decision.mode, decision.wheel, decision.eligible, temp, soc,
                        maxWheel);
}

}  // namespace Orbiedge
