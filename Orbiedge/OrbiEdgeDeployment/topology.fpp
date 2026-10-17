# Not on the configure path. The connections below name ports that
# SimBridge, BusMonitor, FdirManager, and AgentUplinkFilter do not declare.
# The matching sketch is Orbiedge/Topology/OrbiEdgeTopology.fpp.
module Orbiedge {

  deployment topology OrbiEdge {

    instance simBridge
    instance fdirManager
    instance busMonitor
    instance uplinkFilter
    instance rateGroupDriver
    instance rateGroup10Hz
    instance linuxTimer
    instance osTime
    instance cmdDisp
    instance events
    instance textLogger
    instance tlmSend
    instance fatalHandler
    instance fatalAdapter
    instance cmdSeq

    command connections instance cmdDisp

    event connections instance events

    text event connections instance textLogger

    telemetry connections instance tlmSend

    time connections instance osTime

    connections FaultProtection {
      events.FatalAnnounce -> fatalHandler.FatalReceive
    }

    connections RateGroups {
      linuxTimer.CycleOut -> rateGroupDriver.CycleIn
      rateGroupDriver.CycleOut[0] -> rateGroup10Hz.CycleIn
      rateGroup10Hz.RateGroupMemberOut[0] -> simBridge.run
      rateGroup10Hz.RateGroupMemberOut[1] -> fdirManager.run
      rateGroup10Hz.RateGroupMemberOut[2] -> cmdDisp.run
      rateGroup10Hz.RateGroupMemberOut[3] -> events.run
      rateGroup10Hz.RateGroupMemberOut[4] -> cmdSeq.schedIn
    }

    connections Fdir {
      simBridge.timeEpochSet -> osTime.setEpoch
      simBridge.powerOut -> busMonitor.powerIn
      simBridge.thermalOut -> busMonitor.thermalIn
      simBridge.adcsOut -> busMonitor.adcsIn
      simBridge.commitOut -> busMonitor.commitIn
      simBridge.sensorValidOut -> fdirManager.sensorValid
      simBridge.filterSetTime -> uplinkFilter.setTime
      simBridge.proposalOut -> uplinkFilter.proposalIn
      uplinkFilter.acceptOut -> fdirManager.proposalIn
      busMonitor.faultOut -> fdirManager.faultIn
      fdirManager.modeOut -> simBridge.modeIn
      fdirManager.modeEcho -> busMonitor.currentMode
      fdirManager.clearStickyOut -> busMonitor.clearSticky
    }

    connections Sequences {
      cmdSeq.comCmdOut -> cmdDisp.seqCmdBuff[0]
      cmdDisp.seqCmdStatus[0] -> cmdSeq.cmdResponseIn
    }

  }

}
