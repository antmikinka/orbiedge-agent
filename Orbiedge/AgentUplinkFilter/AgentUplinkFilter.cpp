#include "Orbiedge/AgentUplinkFilter/AgentUplinkFilter.hpp"

#include <time.h>

namespace Orbiedge {

AgentUplinkFilter::AgentUplinkFilter(const char* const compName)
    : AgentUplinkFilterComponentBase(compName),
      m_mutex(),
      m_haveTime(false),
      m_simTimeNs(0),
      m_haveAccept(false),
      m_lastAcceptMs(0),
      m_drops(0),
      m_accepts(0) {}

AgentUplinkFilter::~AgentUplinkFilter() {}

U64 AgentUplinkFilter::wallMs() {
    timespec now = {};
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0 || now.tv_sec < 0 || now.tv_nsec < 0) {
        return 0;
    }
    const U64 seconds = static_cast<U64>(now.tv_sec);
    const U64 millis = static_cast<U64>(now.tv_nsec / 1000000L);
    return (seconds * 1000ULL) + millis;
}

void AgentUplinkFilter::drop() {
    this->m_drops = this->m_drops + 1U;
    this->tlmWrite_Drops(this->m_drops);
}

void AgentUplinkFilter::setTime_handler(FwIndexType portNum, U64 simTimeNs) {
    (void)portNum;
    Os::ScopeLock lock(this->m_mutex);
    this->m_haveTime = true;
    this->m_simTimeNs = simTimeNs;
}

void AgentUplinkFilter::proposalIn_handler(FwIndexType portNum,
                                           U32 opcode,
                                           const Mode& action,
                                           U8 wheelIndex,
                                           F32 confidence,
                                           U64 simTimeNs) {
    (void)portNum;
    U64 simNow = 0;
    bool haveTime = false;
    {
        Os::ScopeLock lock(this->m_mutex);
        haveTime = this->m_haveTime;
        simNow = this->m_simTimeNs;
    }
    if (opcode != AgentUplinkFilter::PROPOSE_OPCODE) {
        this->drop();
        return;
    }
    if (haveTime && simNow > simTimeNs && (simNow - simTimeNs) > AgentUplinkFilter::MAX_LAG_NS) {
        this->drop();
        return;
    }
    const U64 nowMs = AgentUplinkFilter::wallMs();
    {
        Os::ScopeLock lock(this->m_mutex);
        if (this->m_haveAccept && nowMs >= this->m_lastAcceptMs &&
            (nowMs - this->m_lastAcceptMs) < AgentUplinkFilter::MIN_WALL_MS) {
            this->m_drops = this->m_drops + 1U;
            this->tlmWrite_Drops(this->m_drops);
            return;
        }
        this->m_haveAccept = true;
        this->m_lastAcceptMs = nowMs;
        this->m_accepts = this->m_accepts + 1U;
        this->tlmWrite_Accepts(this->m_accepts);
    }
    this->acceptOut_out(0, AgentUplinkFilter::PROPOSE_OPCODE, action, wheelIndex, confidence, simTimeNs);
}

}  // namespace Orbiedge
