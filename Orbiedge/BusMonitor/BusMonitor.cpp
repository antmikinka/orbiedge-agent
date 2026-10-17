#include "Orbiedge/BusMonitor/BusMonitor.hpp"

namespace {

Orbiedge::FdirRules::Mode toRules(Orbiedge::Mode mode) {
    switch (mode) {
        case Orbiedge::Mode::NOMINAL:
            return Orbiedge::FdirRules::Mode::NOMINAL;
        case Orbiedge::Mode::ISOLATE_RW:
            return Orbiedge::FdirRules::Mode::ISOLATE_RW;
        case Orbiedge::Mode::MOMENTUM_DUMP:
            return Orbiedge::FdirRules::Mode::MOMENTUM_DUMP;
        case Orbiedge::Mode::LOAD_SHED:
            return Orbiedge::FdirRules::Mode::LOAD_SHED;
        case Orbiedge::Mode::SAFE:
            return Orbiedge::FdirRules::Mode::SAFE;
        default:
            return Orbiedge::FdirRules::Mode::NOMINAL;
    }
}

Orbiedge::Mode fromRules(Orbiedge::FdirRules::Mode mode) {
    switch (mode) {
        case Orbiedge::FdirRules::Mode::NOMINAL:
            return Orbiedge::Mode::NOMINAL;
        case Orbiedge::FdirRules::Mode::ISOLATE_RW:
            return Orbiedge::Mode::ISOLATE_RW;
        case Orbiedge::FdirRules::Mode::MOMENTUM_DUMP:
            return Orbiedge::Mode::MOMENTUM_DUMP;
        case Orbiedge::FdirRules::Mode::LOAD_SHED:
            return Orbiedge::Mode::LOAD_SHED;
        case Orbiedge::FdirRules::Mode::SAFE:
            return Orbiedge::Mode::SAFE;
        default:
            return Orbiedge::Mode::NOMINAL;
    }
}

Orbiedge::Severity fromSeverity(Orbiedge::FdirRules::Severity severity) {
    switch (severity) {
        case Orbiedge::FdirRules::Severity::WARNING:
            return Orbiedge::Severity::WARNING;
        case Orbiedge::FdirRules::Severity::CRITICAL:
            return Orbiedge::Severity::CRITICAL;
        case Orbiedge::FdirRules::Severity::NONE:
        default:
            return Orbiedge::Severity::NONE;
    }
}

}  // namespace

namespace Orbiedge {

BusMonitor::BusMonitor(const char* const compName)
    : BusMonitorComponentBase(compName),
      m_mode(FdirRules::Mode::NOMINAL),
      m_powerSeen(false),
      m_thermalSeen(false),
      m_powerTime(0),
      m_thermalTime(0),
      m_soc(0.0),
      m_solarScale(0.0),
      m_powerValid(false),
      m_tempC(0.0),
      m_heaterOn(false),
      m_thermalValid(false) {}

BusMonitor::~BusMonitor() = default;

void BusMonitor::loadThresholds() {
    Fw::ParamValid valid;
    FdirRules::Thresholds thresholds;
    thresholds.wheel_max_rad_s = static_cast<double>(this->paramGet_WHEEL_MAX_RAD_S(valid));
    thresholds.overspeed_fraction = static_cast<double>(this->paramGet_OVERSPEED_FRACTION(valid));
    thresholds.overspeed_samples = static_cast<int>(this->paramGet_OVERSPEED_SAMPLES(valid));
    thresholds.dump_rise_ns = static_cast<std::uint64_t>(this->paramGet_DUMP_RISE_NS(valid));
    thresholds.stddev_high = static_cast<double>(this->paramGet_STDDEV_HIGH(valid));
    thresholds.stddev_quiet = static_cast<double>(this->paramGet_STDDEV_QUIET(valid));
    thresholds.stddev_samples = static_cast<int>(this->paramGet_STDDEV_SAMPLES(valid));
    thresholds.soc_warn = static_cast<double>(this->paramGet_SOC_WARN(valid));
    thresholds.soc_crit = static_cast<double>(this->paramGet_SOC_CRIT(valid));
    thresholds.soc_samples = static_cast<int>(this->paramGet_SOC_SAMPLES(valid));
    thresholds.temp_warn_c = static_cast<double>(this->paramGet_TEMP_WARN_C(valid));
    thresholds.temp_crit_c = static_cast<double>(this->paramGet_TEMP_CRIT_C(valid));
    thresholds.temp_crit_samples = static_cast<int>(this->paramGet_TEMP_CRIT_SAMPLES(valid));
    thresholds.temp_warn_samples = static_cast<int>(this->paramGet_TEMP_WARN_SAMPLES(valid));
    thresholds.quiet_dwell_ns = static_cast<std::uint64_t>(this->paramGet_QUIET_DWELL_NS(valid));
    this->m_rules.set_thresholds(thresholds);
}

void BusMonitor::emit(const FdirRules::FaultRecord& record) {
    if (!this->isConnected_FaultRecord_OutputPort(0)) {
        return;
    }
    this->FaultRecord_out(0, record.sim_time_ns, record.eligible, fromRules(record.default_mode),
                          fromSeverity(record.severity), record.wheel_index, record.sensor_valid, record.quiet,
                          record.temp_warn_event, record.soc, record.solar_scale, record.temp_c,
                          record.wheel_speed_rad_s[0], record.wheel_speed_rad_s[1], record.wheel_speed_rad_s[2],
                          record.wheel_speed_rad_s[3], record.wheel_stddev[0], record.wheel_stddev[1],
                          record.wheel_stddev[2], record.wheel_stddev[3]);
}

void BusMonitor::powerIn_handler(FwIndexType portNum, U64 simTimeNs, F64 soc, F64 solarScale, bool valid) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->m_powerSeen = true;
    this->m_powerTime = simTimeNs;
    this->m_soc = soc;
    this->m_solarScale = solarScale;
    this->m_powerValid = valid;
}

void BusMonitor::thermalIn_handler(FwIndexType portNum, U64 simTimeNs, F64 tempC, bool heaterOn, bool valid) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->m_thermalSeen = true;
    this->m_thermalTime = simTimeNs;
    this->m_tempC = tempC;
    this->m_heaterOn = heaterOn;
    this->m_thermalValid = valid;
}

void BusMonitor::adcsIn_handler(FwIndexType portNum, U64 simTimeNs, F64 wheel0, F64 wheel1, F64 wheel2, F64 wheel3,
                                bool valid) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    if (!this->m_powerSeen || !this->m_thermalSeen) {
        return;
    }
    this->loadThresholds();
    FdirRules::Sample sample{};
    sample.sim_time_ns = simTimeNs;
    sample.soc = this->m_soc;
    sample.solar_scale = this->m_solarScale;
    sample.temp_c = this->m_tempC;
    sample.heater_on = this->m_heaterOn;
    sample.wheel_speed_rad_s[0] = wheel0;
    sample.wheel_speed_rad_s[1] = wheel1;
    sample.wheel_speed_rad_s[2] = wheel2;
    sample.wheel_speed_rad_s[3] = wheel3;
    const bool aligned = this->m_powerTime == simTimeNs && this->m_thermalTime == simTimeNs;
    sample.valid = valid && this->m_powerValid && this->m_thermalValid && aligned;
    const FdirRules::FaultRecord record = this->m_rules.observe(sample, this->m_mode);
    this->m_powerSeen = false;
    this->m_thermalSeen = false;
    this->emit(record);
}

void BusMonitor::modeIn_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) {
    (void)portNum;
    (void)simTimeNs;
    (void)wheelIndex;
    Os::ScopeLock lock(this->m_mutex);
    this->m_mode = toRules(mode);
}

void BusMonitor::clearIn_handler(FwIndexType portNum, bool reset) {
    (void)portNum;
    if (!reset) {
        return;
    }
    Os::ScopeLock lock(this->m_mutex);
    this->m_rules.clear_monitor();
    this->m_mode = FdirRules::Mode::NOMINAL;
}

}  // namespace Orbiedge
