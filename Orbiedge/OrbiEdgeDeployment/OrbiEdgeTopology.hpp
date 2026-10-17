#ifndef ORBIEDGE_ORBIEDGETOPOLOGY_HPP
#define ORBIEDGE_ORBIEDGETOPOLOGY_HPP

#include "Fw/Time/TimeInterval.hpp"
#include "Orbiedge/OrbiEdgeDeployment/OrbiEdgeTopologyDefs.hpp"

namespace Orbiedge {

void setupTopology(const TopologyState& state);
void startRateGroups(const Fw::TimeInterval& interval);
void stopRateGroups();
void teardownTopology(const TopologyState& state);

}  // namespace Orbiedge

#endif
