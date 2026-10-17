#include "Orbiedge/OrbiEdgeDeployment/OrbiEdgeTopology.hpp"

#include "Fw/Logger/Logger.hpp"
#include "Fw/Time/TimeInterval.hpp"
#include "Os/Os.hpp"

#include <csignal>

static void signalHandler(int signum) {
    (void)signum;
    Orbiedge::stopRateGroups();
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    Os::init();
    Orbiedge::TopologyState state = {};
    state.unused = 0;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    Fw::Logger::log(
        "OrbiEdge FDIR deployment. Sensor client 127.0.0.1:5600, mode server 127.0.0.1:5601, "
        "advisor server 127.0.0.1:50001. Human GDS port 50000 is not opened. Ctrl-C quits.\n");
    Orbiedge::setupTopology(state);
    Orbiedge::startRateGroups(Fw::TimeInterval(0, 100000));
    Orbiedge::teardownTopology(state);
    Fw::Logger::log("Exiting.\n");
    return 0;
}
