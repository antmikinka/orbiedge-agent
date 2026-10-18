#ifndef ORBIEDGE_AGENTUPLINKFILTER_HPP
#define ORBIEDGE_AGENTUPLINKFILTER_HPP

#include "Orbiedge/AgentUplinkFilter/AgentUplinkFilterComponentAc.hpp"

#include "Os/Mutex.hpp"

namespace Orbiedge {

class AgentUplinkFilter final : public AgentUplinkFilterComponentBase {
  public:
    explicit AgentUplinkFilter(const char* const compName);
    ~AgentUplinkFilter() override;

  private:
    static constexpr U32 PROPOSE_OPCODE = 0;
    static constexpr U64 MAX_LAG_NS = 5000000000ULL;
    static constexpr U64 MIN_WALL_MS = 2000ULL;

    void proposalIn_handler(FwIndexType portNum, U32 opcode, const Mode& action, U8 wheelIndex, F32 confidence,
                            U64 simTimeNs) override;
    void setTime_handler(FwIndexType portNum, U64 simTimeNs) override;

    void drop();
    static U64 wallMs();

    Os::Mutex m_mutex;
    bool m_haveTime;
    U64 m_simTimeNs;
    bool m_haveAccept;
    U64 m_lastAcceptMs;
    U32 m_drops;
    U32 m_accepts;
};

}  // namespace Orbiedge

#endif
