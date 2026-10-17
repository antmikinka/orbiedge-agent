#include "Orbiedge/AgentUplinkFilter/AgentUplinkFilter.hpp"

namespace Orbiedge {

AgentUplinkFilter::AgentUplinkFilter(const char* const compName)
    : AgentUplinkFilterComponentBase(compName), m_dropped(0) {}

AgentUplinkFilter::~AgentUplinkFilter() = default;

void AgentUplinkFilter::drop() {
    this->m_dropped += 1;
    this->tlmWrite_CommandsDropped(this->m_dropped);
}

void AgentUplinkFilter::uplinkIn_handler(FwIndexType portNum, const Fw::StringBase& opcode, const Mode& action,
                                         U8 wheelIndex, F32 confidence, U64 simTimeNs, U64 nowSimTimeNs,
                                         U64 wallTimeNs) {
    const FdirRules::FilterResult result =
        this->m_filter.screen(opcode.toChar(), simTimeNs, nowSimTimeNs, wallTimeNs, wheelIndex, confidence);
    if (result != FdirRules::FilterResult::PASS) {
        this->drop();
        return;
    }
    if (this->isConnected_forwardOut_OutputPort(portNum)) {
        this->forwardOut_out(portNum, opcode, action, wheelIndex, confidence, simTimeNs, nowSimTimeNs, wallTimeNs);
    } else if (this->isConnected_forwardOut_OutputPort(0)) {
        this->forwardOut_out(0, opcode, action, wheelIndex, confidence, simTimeNs, nowSimTimeNs, wallTimeNs);
    }
}

}  // namespace Orbiedge
