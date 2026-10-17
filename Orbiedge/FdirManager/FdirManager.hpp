#ifndef ORBIEDGE_FDIRMANAGER_HPP
#define ORBIEDGE_FDIRMANAGER_HPP

#include "Orbiedge/FdirManager/FdirManagerComponentAc.hpp"
#include "Orbiedge/FdirRules/FdirRules.hpp"

#include <Os/Mutex.hpp>

namespace Orbiedge {

// Queued. Tick rules run when the async schedIn message is dispatched.
// There is no wheel-torque port.
class FdirManager final : public FdirManagerComponentBase {
  public:
    explicit FdirManager(const char* const compName);
    ~FdirManager() override;

  private:
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void faultIn_handler(FwIndexType portNum, U64 simTimeNs, U8 eligible, const Mode& defaultMode,
                         const Severity& severity, U8 wheelIndex, bool sensorValid, bool quiet, bool tempWarn, F64 soc,
                         F64 solarScale, F64 tempC, F64 wheel0, F64 wheel1, F64 wheel2, F64 wheel3, F64 std0, F64 std1,
                         F64 std2, F64 std3) override;
    void AGENT_PROPOSE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Mode& action, U8 wheel_index, F32 confidence,
                                  U64 sim_time_ns) override;
    void FDIR_CLEAR_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    void publish(const FdirRules::Decision& decision, const FdirRules::FaultRecord& record);

    Os::Mutex m_mutex;
    FdirRules m_rules;
    FdirRules::FaultRecord m_record;
    bool m_haveRecord;
    FdirRules::Proposal m_proposal;
    bool m_haveProposal;
    bool m_havePublished;
    Mode m_publishedMode;
    bool m_sent;
    Mode m_sentMode;
    U8 m_sentWheel;
    bool m_holdLatched;
    bool m_tempWarnLatched;
};

}  // namespace Orbiedge

#endif
