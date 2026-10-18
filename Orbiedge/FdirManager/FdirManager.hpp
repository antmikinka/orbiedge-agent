#ifndef ORBIEDGE_FDIRMANAGER_HPP
#define ORBIEDGE_FDIRMANAGER_HPP

#include "Orbiedge/FdirManager/FdirManagerComponentAc.hpp"

#include "Os/Mutex.hpp"

namespace Orbiedge {

class FdirManager final : public FdirManagerComponentBase {
  public:
    explicit FdirManager(const char* const compName);
    ~FdirManager() override;

  private:
    static constexpr U64 WINDOW_NS = 2000000000ULL;
    static constexpr U64 QUIET_NS = 60000000000ULL;

    struct Fault {
        bool fresh;
        U64 simTimeNs;
        Severity severity;
        Mode modeDefault;
        U8 wheelIndex;
        U8 eligibleCount;
        F64 tempC;
        F64 soc;
        F64 maxWheel;
    };

    struct Proposal {
        bool pending;
        Mode action;
        U8 wheelIndex;
        F32 confidence;
        U64 simTimeNs;
    };

    void run_handler(FwIndexType portNum, U32 context) override;
    void faultIn_handler(FwIndexType portNum,
                         U64 simTimeNs,
                         const Severity& faultSeverity,
                         const Mode& modeDefault,
                         U8 wheelIndex,
                         U8 eligibleCount,
                         F64 tempC,
                         F64 soc,
                         F64 maxWheel) override;
    void sensorValid_handler(FwIndexType portNum, bool valid) override;
    void proposalIn_handler(FwIndexType portNum, U32 opcode, const Mode& action, U8 wheelIndex, F32 confidence,
                            U64 simTimeNs) override;
    void AGENT_PROPOSE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Mode& action, U8 wheelIndex, F32 confidence,
                                  U64 simTimeNs) override;
    void FDIR_CLEAR_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    void storeProposal(Mode action, U8 wheelIndex, F32 confidence, U64 simTimeNs);
    void applyTick(const Fault& fault, bool sensorValid, Proposal& proposal);
    bool actionAllowed(Mode modeDefault, U8 eligibleCount, Mode action) const;
    void setMode(U64 simTimeNs, Mode mode, U8 wheelIndex);
    void noteDisposition(Disposition disposition, Mode action);

    Os::Mutex m_mutex;
    Fault m_fault;
    Proposal m_proposal;
    bool m_sensorValid;
    bool m_haveSensor;
    Mode m_mode;
    U8 m_wheel;
    bool m_stickySafe;
    bool m_stickyIsolate;
    bool m_windowOpen;
    U64 m_windowStart;
    Mode m_windowDefault;
    bool m_quietTiming;
    U64 m_quietSince;
    Disposition m_disposition;
    Mode m_lastProposal;
};

}  // namespace Orbiedge

#endif
