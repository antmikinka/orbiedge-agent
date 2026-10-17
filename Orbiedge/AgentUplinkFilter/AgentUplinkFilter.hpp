#ifndef ORBIEDGE_AGENTUPLINKFILTER_HPP
#define ORBIEDGE_AGENTUPLINKFILTER_HPP

#include "Orbiedge/AgentUplinkFilter/AgentUplinkFilterComponentAc.hpp"
#include "Orbiedge/FdirRules/FdirRules.hpp"

namespace Orbiedge {

// Sits in front of the dispatcher. Passes one opcode: AGENT.PROPOSE.
class AgentUplinkFilter final : public AgentUplinkFilterComponentBase {
  public:
    explicit AgentUplinkFilter(const char* const compName);
    ~AgentUplinkFilter() override;

  private:
    void uplinkIn_handler(FwIndexType portNum, const Fw::StringBase& opcode, const Mode& action, U8 wheelIndex,
                          F32 confidence, U64 simTimeNs, U64 nowSimTimeNs, U64 wallTimeNs) override;

    void drop();

    FdirRules::ProposalFilter m_filter;
    U32 m_dropped;
};

}  // namespace Orbiedge

#endif
