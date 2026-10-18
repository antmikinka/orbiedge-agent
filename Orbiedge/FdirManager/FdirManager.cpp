#include "Orbiedge/FdirManager/FdirManager.hpp"

namespace Orbiedge {

FdirManager::FdirManager(const char* const compName)
    : FdirManagerComponentBase(compName),
      m_mutex(),
      m_fault{},
      m_proposal{},
      m_sensorValid(false),
      m_haveSensor(false),
      m_mode(Mode::NOMINAL),
      m_wheel(0),
      m_stickySafe(false),
      m_stickyIsolate(false),
      m_windowOpen(false),
      m_windowStart(0),
      m_windowDefault(Mode::NOMINAL),
      m_quietTiming(false),
      m_quietSince(0),
      m_disposition(Disposition::NONE),
      m_lastProposal(Mode::NOMINAL) {
    this->m_fault.fresh = false;
    this->m_fault.severity = Severity::NONE;
    this->m_fault.modeDefault = Mode::NOMINAL;
    this->m_proposal.pending = false;
    this->m_proposal.action = Mode::NOMINAL;
}

FdirManager::~FdirManager() {}

void FdirManager::faultIn_handler(FwIndexType portNum,
                                  U64 simTimeNs,
                                  const Severity& faultSeverity,
                                  const Mode& modeDefault,
                                  U8 wheelIndex,
                                  U8 eligibleCount,
                                  F64 tempC,
                                  F64 soc,
                                  F64 maxWheel) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->m_fault.fresh = true;
    this->m_fault.simTimeNs = simTimeNs;
    this->m_fault.severity = faultSeverity;
    this->m_fault.modeDefault = modeDefault;
    this->m_fault.wheelIndex = wheelIndex;
    this->m_fault.eligibleCount = eligibleCount;
    this->m_fault.tempC = tempC;
    this->m_fault.soc = soc;
    this->m_fault.maxWheel = maxWheel;
}

void FdirManager::sensorValid_handler(FwIndexType portNum, bool valid) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->m_haveSensor = true;
    this->m_sensorValid = valid;
}

void FdirManager::storeProposal(Mode action, U8 wheelIndex, F32 confidence, U64 simTimeNs) {
    Os::ScopeLock lock(this->m_mutex);
    this->m_proposal.pending = true;
    this->m_proposal.action = action;
    this->m_proposal.wheelIndex = wheelIndex;
    this->m_proposal.confidence = confidence;
    this->m_proposal.simTimeNs = simTimeNs;
    this->m_lastProposal = action;
}

void FdirManager::proposalIn_handler(FwIndexType portNum,
                                     U32 opcode,
                                     const Mode& action,
                                     U8 wheelIndex,
                                     F32 confidence,
                                     U64 simTimeNs) {
    (void)portNum;
    if (opcode != 0U) {
        this->noteDisposition(Disposition::REJECT, action);
        return;
    }
    this->storeProposal(action, wheelIndex, confidence, simTimeNs);
}

void FdirManager::AGENT_PROPOSE_cmdHandler(FwOpcodeType opCode,
                                           U32 cmdSeq,
                                           const Mode& action,
                                           U8 wheelIndex,
                                           F32 confidence,
                                           U64 simTimeNs) {
    this->storeProposal(action, wheelIndex, confidence, simTimeNs);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FdirManager::FDIR_CLEAR_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    {
        Os::ScopeLock lock(this->m_mutex);
        this->m_stickySafe = false;
        this->m_stickyIsolate = false;
        this->m_windowOpen = false;
    }
    this->clearStickyOut_out(0, 1);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

bool FdirManager::actionAllowed(Mode modeDefault, U8 eligibleCount, Mode action) const {
    if (eligibleCount <= 1U) {
        return action == modeDefault;
    }
    if (modeDefault == Mode::MOMENTUM_DUMP) {
        return action == Mode::MOMENTUM_DUMP || action == Mode::SAFE;
    }
    if (modeDefault == Mode::ISOLATE_RW) {
        return action == Mode::ISOLATE_RW || action == Mode::NOMINAL;
    }
    return action == modeDefault;
}

void FdirManager::noteDisposition(Disposition disposition, Mode action) {
    this->m_disposition = disposition;
    this->m_lastProposal = action;
    this->log_ACTIVITY_LO_ProposalResult(disposition, action);
}

void FdirManager::setMode(U64 simTimeNs, Mode mode, U8 wheelIndex) {
    const Mode previous = this->m_mode;
    this->m_mode = mode;
    this->m_wheel = wheelIndex;
    if (mode == Mode::SAFE) {
        this->m_stickySafe = true;
    }
    if (mode == Mode::ISOLATE_RW) {
        this->m_stickyIsolate = true;
    }
    if (previous != mode) {
        this->log_ACTIVITY_HI_ModeTransition(previous, mode);
    }
    this->tlmWrite_Mode(mode);
    this->modeOut_out(0, simTimeNs, mode, wheelIndex);
    this->modeEcho_out(0, simTimeNs, mode, wheelIndex);
}

void FdirManager::applyTick(const Fault& fault, bool sensorValid, Proposal& proposal) {
    this->tlmWrite_SensorValid(sensorValid);
    this->tlmWrite_SimTimeNs(fault.simTimeNs);
    this->tlmWrite_Soc(fault.soc);
    this->tlmWrite_TempC(fault.tempC);
    this->tlmWrite_Severity(fault.severity);
    this->tlmWrite_EligibleCount(fault.eligibleCount);
    this->tlmWrite_Mode(this->m_mode);
    this->tlmWrite_LastProposal(this->m_lastProposal);
    this->tlmWrite_ProposalDisposition(this->m_disposition);

    if (!sensorValid) {
        this->m_disposition = Disposition::HOLD_INVALID;
        this->tlmWrite_ProposalDisposition(this->m_disposition);
        this->m_windowOpen = false;
        this->m_quietTiming = false;
        return;
    }
    if (!fault.fresh) {
        return;
    }

    if (this->m_stickySafe) {
        this->setMode(fault.simTimeNs, Mode::SAFE, this->m_wheel);
        proposal.pending = false;
        return;
    }

    if (fault.severity == Severity::CRITICAL) {
        this->m_windowOpen = false;
        this->m_quietTiming = false;
        U8 wheel = fault.wheelIndex;
        if (proposal.pending) {
            this->noteDisposition(Disposition::DISAGREE, proposal.action);
            proposal.pending = false;
        }
        this->setMode(fault.simTimeNs, fault.modeDefault, wheel);
        return;
    }

    if (fault.severity == Severity::WARNING && fault.eligibleCount <= 1U) {
        this->m_windowOpen = false;
        this->m_quietTiming = false;
        if (proposal.pending) {
            const Disposition disposition =
                this->actionAllowed(fault.modeDefault, fault.eligibleCount, proposal.action) ? Disposition::AGREE
                                                                                              : Disposition::DISAGREE;
            this->noteDisposition(disposition, proposal.action);
            proposal.pending = false;
        }
        this->setMode(fault.simTimeNs, fault.modeDefault, fault.wheelIndex);
        return;
    }

    if (fault.severity == Severity::WARNING && fault.eligibleCount > 1U) {
        this->m_quietTiming = false;
        if (!this->m_windowOpen || this->m_windowDefault != fault.modeDefault) {
            this->m_windowOpen = true;
            this->m_windowStart = fault.simTimeNs;
            this->m_windowDefault = fault.modeDefault;
        }
        const bool expired =
            fault.simTimeNs >= this->m_windowStart && (fault.simTimeNs - this->m_windowStart) >= FdirManager::WINDOW_NS;
        if (proposal.pending) {
            const bool confidenceOk = proposal.confidence >= 0.0F && proposal.confidence <= 1.0F;
            const bool wheelOk = proposal.wheelIndex <= 3U;
            const bool inWindow = proposal.simTimeNs >= this->m_windowStart &&
                                  proposal.simTimeNs <= (this->m_windowStart + FdirManager::WINDOW_NS);
            if (!confidenceOk || !wheelOk) {
                this->noteDisposition(Disposition::REJECT, proposal.action);
            } else if (!inWindow) {
                this->noteDisposition(Disposition::LATE, proposal.action);
            } else if (!this->actionAllowed(fault.modeDefault, fault.eligibleCount, proposal.action)) {
                this->noteDisposition(Disposition::DISAGREE, proposal.action);
            } else if (this->m_stickyIsolate && proposal.action != Mode::ISOLATE_RW) {
                this->noteDisposition(Disposition::DISAGREE, proposal.action);
            } else {
                this->noteDisposition(Disposition::AGREE, proposal.action);
                U8 wheel = fault.wheelIndex;
                if (proposal.action == Mode::ISOLATE_RW) {
                    wheel = proposal.wheelIndex;
                }
                this->setMode(fault.simTimeNs, proposal.action, wheel);
                this->m_windowOpen = false;
                proposal.pending = false;
                return;
            }
            proposal.pending = false;
        }
        if (expired) {
            this->setMode(fault.simTimeNs, fault.modeDefault, fault.wheelIndex);
            this->m_windowOpen = false;
        }
        return;
    }

    this->m_windowOpen = false;
    if (this->m_stickyIsolate) {
        this->setMode(fault.simTimeNs, Mode::ISOLATE_RW, this->m_wheel);
        return;
    }
    if (!this->m_quietTiming) {
        this->m_quietTiming = true;
        this->m_quietSince = fault.simTimeNs;
        return;
    }
    if (fault.simTimeNs >= this->m_quietSince && (fault.simTimeNs - this->m_quietSince) >= FdirManager::QUIET_NS &&
        this->m_mode != Mode::NOMINAL) {
        this->setMode(fault.simTimeNs, Mode::NOMINAL, 0);
    }
}

void FdirManager::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;
    Fault fault;
    Proposal proposal;
    bool sensorValid = false;
    {
        Os::ScopeLock lock(this->m_mutex);
        fault = this->m_fault;
        proposal = this->m_proposal;
        sensorValid = this->m_sensorValid;
        if (sensorValid) {
            this->m_proposal.pending = false;
        }
    }
    this->applyTick(fault, sensorValid, proposal);
}

}  // namespace Orbiedge
