#include "Orbiedge/FdirManager/FdirManager.hpp"

namespace {

Orbiedge::FdirRules::Mode toRules(Orbiedge::Mode mode) {
    switch (mode) {
        case Orbiedge::Mode::ISOLATE_RW:
            return Orbiedge::FdirRules::Mode::ISOLATE_RW;
        case Orbiedge::Mode::MOMENTUM_DUMP:
            return Orbiedge::FdirRules::Mode::MOMENTUM_DUMP;
        case Orbiedge::Mode::LOAD_SHED:
            return Orbiedge::FdirRules::Mode::LOAD_SHED;
        case Orbiedge::Mode::SAFE:
            return Orbiedge::FdirRules::Mode::SAFE;
        case Orbiedge::Mode::NOMINAL:
        default:
            return Orbiedge::FdirRules::Mode::NOMINAL;
    }
}

Orbiedge::Mode fromRules(Orbiedge::FdirRules::Mode mode) {
    switch (mode) {
        case Orbiedge::FdirRules::Mode::ISOLATE_RW:
            return Orbiedge::Mode::ISOLATE_RW;
        case Orbiedge::FdirRules::Mode::MOMENTUM_DUMP:
            return Orbiedge::Mode::MOMENTUM_DUMP;
        case Orbiedge::FdirRules::Mode::LOAD_SHED:
            return Orbiedge::Mode::LOAD_SHED;
        case Orbiedge::FdirRules::Mode::SAFE:
            return Orbiedge::Mode::SAFE;
        case Orbiedge::FdirRules::Mode::NOMINAL:
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

Orbiedge::FdirRules::Severity toSeverity(Orbiedge::Severity severity) {
    switch (severity) {
        case Orbiedge::Severity::WARNING:
            return Orbiedge::FdirRules::Severity::WARNING;
        case Orbiedge::Severity::CRITICAL:
            return Orbiedge::FdirRules::Severity::CRITICAL;
        case Orbiedge::Severity::NONE:
        default:
            return Orbiedge::FdirRules::Severity::NONE;
    }
}

Orbiedge::Disposition fromDisposition(Orbiedge::FdirRules::Disposition disposition) {
    switch (disposition) {
        case Orbiedge::FdirRules::Disposition::AGREE:
            return Orbiedge::Disposition::AGREE;
        case Orbiedge::FdirRules::Disposition::DISAGREE:
            return Orbiedge::Disposition::DISAGREE;
        case Orbiedge::FdirRules::Disposition::LATE:
            return Orbiedge::Disposition::LATE;
        case Orbiedge::FdirRules::Disposition::REJECT:
            return Orbiedge::Disposition::REJECT;
        case Orbiedge::FdirRules::Disposition::HOLD_INVALID:
            return Orbiedge::Disposition::HOLD_INVALID;
        case Orbiedge::FdirRules::Disposition::NONE:
        default:
            return Orbiedge::Disposition::NONE;
    }
}

}  // namespace

namespace Orbiedge {

FdirManager::FdirManager(const char* const compName)
    : FdirManagerComponentBase(compName),
      m_haveRecord(false),
      m_haveProposal(false),
      m_havePublished(false),
      m_publishedMode(Mode::NOMINAL),
      m_sent(false),
      m_sentMode(Mode::NOMINAL),
      m_sentWheel(0),
      m_holdLatched(false),
      m_tempWarnLatched(false) {
    this->m_proposal.opcode = FdirRules::OPCODE_PROPOSE;
    this->m_proposal.action = FdirRules::Mode::NOMINAL;
    this->m_proposal.wheel_index = 0;
    this->m_proposal.confidence = 0.0f;
    this->m_proposal.sim_time_ns = 0;
    this->m_proposal.wall_time_ns = 0;
}

FdirManager::~FdirManager() = default;

void FdirManager::faultIn_handler(FwIndexType portNum, U64 simTimeNs, U8 eligible, const Mode& defaultMode,
                                  const Severity& severity, U8 wheelIndex, bool sensorValid, bool quiet, bool tempWarn,
                                  F64 soc, F64 solarScale, F64 tempC, F64 wheel0, F64 wheel1, F64 wheel2, F64 wheel3,
                                  F64 std0, F64 std1, F64 std2, F64 std3) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    FdirRules::FaultRecord record{};
    record.sim_time_ns = simTimeNs;
    record.eligible = eligible;
    record.default_mode = toRules(defaultMode);
    record.severity = toSeverity(severity);
    record.wheel_index = wheelIndex;
    record.sensor_valid = sensorValid;
    record.quiet = quiet;
    record.temp_warn_event = tempWarn;
    record.soc = soc;
    record.solar_scale = solarScale;
    record.temp_c = tempC;
    record.wheel_speed_rad_s[0] = wheel0;
    record.wheel_speed_rad_s[1] = wheel1;
    record.wheel_speed_rad_s[2] = wheel2;
    record.wheel_speed_rad_s[3] = wheel3;
    record.wheel_stddev[0] = std0;
    record.wheel_stddev[1] = std1;
    record.wheel_stddev[2] = std2;
    record.wheel_stddev[3] = std3;
    this->m_record = record;
    this->m_haveRecord = true;
}

void FdirManager::AGENT_PROPOSE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Mode& action, U8 wheel_index,
                                           F32 confidence, U64 sim_time_ns) {
    if (wheel_index > 3 || !(confidence >= 0.0f && confidence <= 1.0f)) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    {
        Os::ScopeLock lock(this->m_mutex);
        this->m_proposal.opcode = FdirRules::OPCODE_PROPOSE;
        this->m_proposal.action = toRules(action);
        this->m_proposal.wheel_index = wheel_index;
        this->m_proposal.confidence = confidence;
        this->m_proposal.sim_time_ns = sim_time_ns;
        this->m_proposal.wall_time_ns = 0;
        this->m_haveProposal = true;
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FdirManager::FDIR_CLEAR_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->m_rules.clear_manager();
    if (this->isConnected_clearOut_OutputPort(0)) {
        this->clearOut_out(0, true);
    }
    this->m_havePublished = true;
    if (this->m_publishedMode != Mode::NOMINAL) {
        this->log_ACTIVITY_HI_ModeTransition(this->m_publishedMode, Mode::NOMINAL, 0);
    }
    this->m_publishedMode = Mode::NOMINAL;
    this->m_sent = true;
    this->m_sentMode = Mode::NOMINAL;
    this->m_sentWheel = 0;
    if (this->isConnected_modeOut_OutputPort(0)) {
        this->modeOut_out(0, 0, Mode::NOMINAL, 0);
    }
    if (this->isConnected_modeToMonitor_OutputPort(0)) {
        this->modeToMonitor_out(0, 0, Mode::NOMINAL, 0);
    }
    this->tlmWrite_MODE(Mode::NOMINAL);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FdirManager::schedIn_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;
    FdirRules::FaultRecord record{};
    FdirRules::Proposal proposal{};
    bool have_record = false;
    bool have_proposal = false;
    {
        Os::ScopeLock lock(this->m_mutex);
        have_record = this->m_haveRecord;
        record = this->m_record;
        have_proposal = this->m_haveProposal;
        proposal = this->m_proposal;
        this->m_haveProposal = false;
    }
    if (!have_record) {
        this->tlmWrite_SENSOR_VALID(false);
        return;
    }
    const FdirRules::Decision decision = this->m_rules.decide(record, have_proposal ? &proposal : nullptr);
    this->publish(decision, record);
}

void FdirManager::publish(const FdirRules::Decision& decision, const FdirRules::FaultRecord& record) {
    const Mode mode = fromRules(decision.mode);
    const Mode last = fromRules(decision.last_proposal);
    WheelF64 speeds;
    WheelF64 deviation;
    for (int wheel = 0; wheel < 4; ++wheel) {
        speeds[wheel] = record.wheel_speed_rad_s[wheel];
        deviation[wheel] = record.wheel_stddev[wheel];
    }
    this->tlmWrite_MODE(mode);
    this->tlmWrite_SEVERITY(fromSeverity(decision.severity));
    this->tlmWrite_ELIGIBLE(decision.eligible);
    this->tlmWrite_SOC(record.soc);
    this->tlmWrite_SOLAR_SCALE(record.solar_scale);
    this->tlmWrite_WHEEL_SPEED(speeds);
    this->tlmWrite_WHEEL_STDDEV(deviation);
    this->tlmWrite_TEMP_C(record.temp_c);
    this->tlmWrite_SENSOR_VALID(decision.sensor_valid);
    this->tlmWrite_LAST_PROPOSAL(last);
    this->tlmWrite_PROPOSAL_DISPOSITION(fromDisposition(decision.disposition));
    this->tlmWrite_SIM_TIME_NS(decision.sim_time_ns);

    if (!this->m_havePublished) {
        this->m_havePublished = true;
        this->m_publishedMode = mode;
        if (mode != Mode::NOMINAL) {
            this->log_ACTIVITY_HI_ModeTransition(Mode::NOMINAL, mode, decision.sim_time_ns);
        }
    } else if (mode != this->m_publishedMode) {
        this->log_ACTIVITY_HI_ModeTransition(this->m_publishedMode, mode, decision.sim_time_ns);
        this->m_publishedMode = mode;
    }

    if (decision.disposition == FdirRules::Disposition::HOLD_INVALID) {
        if (!this->m_holdLatched) {
            this->m_holdLatched = true;
            this->log_WARNING_LO_HoldInvalid(decision.sim_time_ns);
        }
    } else {
        this->m_holdLatched = false;
    }

    if (decision.disposition != FdirRules::Disposition::NONE &&
        decision.disposition != FdirRules::Disposition::HOLD_INVALID) {
        this->log_ACTIVITY_LO_ProposalScored(fromDisposition(decision.disposition), last);
    }

    if (decision.temp_warn_event && !this->m_tempWarnLatched) {
        this->m_tempWarnLatched = true;
        this->log_ACTIVITY_LO_TempWarn(record.temp_c);
    }
    if (!decision.temp_warn_event) {
        this->m_tempWarnLatched = false;
    }

    if (!this->m_sent || mode != this->m_sentMode || decision.wheel_index != this->m_sentWheel) {
        this->m_sent = true;
        this->m_sentMode = mode;
        this->m_sentWheel = decision.wheel_index;
        if (this->isConnected_modeOut_OutputPort(0)) {
            this->modeOut_out(0, decision.sim_time_ns, mode, decision.wheel_index);
        }
        if (this->isConnected_modeToMonitor_OutputPort(0)) {
            this->modeToMonitor_out(0, decision.sim_time_ns, mode, decision.wheel_index);
        }
    }
}

}  // namespace Orbiedge
