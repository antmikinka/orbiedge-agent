module Orbiedge {

  @ Only mode authority. Critical faults apply on this tick. Proposals cannot clear
  @ a sticky SAFE or sticky ISOLATE_RW. The 2 second warning window is sim time.
  active component FdirManager {

    async input port run: Svc.Sched drop

    sync input port faultIn: FaultRecord

    sync input port sensorValid: SensorValid

    sync input port proposalIn: Proposal

    output port modeOut: ModeRequest

    output port modeEcho: ModeRequest

    output port clearStickyOut: ClearSticky

    async command AGENT_PROPOSE(
      requestedMode: Mode
      wheelIndex: U8
      confidence: F32
      simTimeNs: U64
    ) opcode 0

    async command FDIR_CLEAR opcode 1

    telemetry Mode: Mode id 0

    telemetry Severity: Severity id 1

    telemetry EligibleCount: U8 id 2

    telemetry Soc: F64 id 3

    telemetry TempC: F64 id 4

    telemetry SensorValid: bool id 5

    telemetry LastProposal: Mode id 6

    telemetry ProposalDisposition: Disposition id 7

    telemetry SimTimeNs: U64 id 8

    event ModeTransition(
      fromMode: Mode
      toMode: Mode
    ) severity activity high id 0 format "FdirManager mode {} -> {}"

    event ProposalResult(
      disposition: Disposition
      requestedMode: Mode
    ) severity activity low id 1 format "Proposal {} action {}"

    import Fw.Command

    import Fw.Event

    import Fw.Channel

    time get port timeGetOut

  }

}
