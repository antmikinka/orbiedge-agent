#include "Orbiedge/OrbiEdgeDeployment/OrbiEdgeTopology.hpp"

#include "Fw/Types/MallocAllocator.hpp"
#include "Orbiedge/OrbiEdgeDeployment/OrbiEdgeTopologyAc.hpp"

using namespace Orbiedge;

namespace {

Fw::MallocAllocator mallocAllocator;
Svc::RateGroupDriver::DividerSet rateDividers{{{1, 0}, {2, 0}, {4, 0}}};
Svc::ActiveRateGroup::ContextArray rateContext(0);

void configureTopology() {
    rateGroupDriver.configure(rateDividers);
    rateGroup10Hz.configure(rateContext);
    cmdSeq.allocateBuffer(0, mallocAllocator, 5 * 1024);
    simBridge.startSockets();
}

}  // namespace

namespace Orbiedge {

void setupTopology(const TopologyState& state) {
    initComponents(state);
    setBaseIds();
    connectComponents();
    configComponents(state);
    configureTopology();
    regCommands();
    startTasks(state);
}

void startRateGroups(const Fw::TimeInterval& interval) {
    linuxTimer.startTimer(interval);
}

void stopRateGroups() {
    linuxTimer.quit();
}

void teardownTopology(const TopologyState& state) {
    stopTasks(state);
    freeThreads(state);
    cmdSeq.deallocateBuffer(mallocAllocator);
    tearDownComponents(state);
    deinitComponents(state);
}

}  // namespace Orbiedge
