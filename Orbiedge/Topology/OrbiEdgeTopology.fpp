module Orbiedge {

  constant QUEUE_SIZE = 10
  constant STACK_SIZE = 64 * 1024

  instance simBridge: Orbiedge.SimBridge base id 0xE1000000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 40

  instance busMonitor: Orbiedge.BusMonitor base id 0xE2000000

  instance fdirManager: Orbiedge.FdirManager base id 0xE3000000 \
    queue size QUEUE_SIZE

  instance agentUplinkFilter: Orbiedge.AgentUplinkFilter base id 0xE4000000

  @ Four project components only. This is not a copy of the Ref deployment.
  topology OrbiEdgeTopology {

    instance simBridge

    instance busMonitor

    instance fdirManager

    instance agentUplinkFilter

    connections Sensors {
      simBridge.PowerState -> busMonitor.powerIn
      simBridge.ThermalState -> busMonitor.thermalIn
      simBridge.AdcsState -> busMonitor.adcsIn
      busMonitor.FaultRecord -> fdirManager.faultIn
      fdirManager.modeOut -> simBridge.modeIn
      fdirManager.modeToMonitor -> busMonitor.modeIn
      fdirManager.clearOut -> busMonitor.clearIn
    }

    # 10 Hz rate group (not instanced here; do not copy Ref):
    #   rateGroup10Hz.RateGroupMemberOut[0] -> simBridge.schedIn
    #   rateGroup10Hz.RateGroupMemberOut[1] -> fdirManager.schedIn
    # Both inputs are async drop. The rate group enqueues and does not block.
    # FdirManager is queued, not active. Its schedIn message runs when that queue is dispatched.
    # SimBridge is active, so its thread owns the 127.0.0.1:5600 client and the :5601 writer.

    # Human GDS stays on Drv.TcpServer 127.0.0.1:50000 and CmdDispatcher source port 0.
    # Advisor uplink is a second Drv.TcpServer on 127.0.0.1:50001, deframed into
    # agentUplinkFilter.uplinkIn. forwardOut is the only path to CmdDispatcher source port 1.
    # FDIR.CLEAR is not forwarded on source port 1. Bind addresses are 127.0.0.1 only.

    # Time-set port: Svc.Time in Svc/Interfaces/Time.fpp declares only timeGetPort.
    # Svc.OsTime has setEpoch (OsTimeEpoch) and command SetCurrentTime. Those are not
    # a generic external time-set port, and this topology does not call a guessed name.
    # Connect the real time-set port when the pinned SDD names it.
    # Until then, Basilisk sim_time_ns is the typed argument of stampSimTime and the sensor ports.

  }

}
