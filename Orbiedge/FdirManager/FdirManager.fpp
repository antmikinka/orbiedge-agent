module Orbiedge {

  array WheelF64 = [4] F64

  @ Queued. schedIn is async drop on the 10 Hz group so a late tick is discarded.
  @ No wheel-torque port. Basilisk keeps the stand-in controller.
  @ Dictionary strings are AGENT.PROPOSE and FDIR.CLEAR. FPP identifiers have no dot.
  queued component FdirManager {

    async input port schedIn: Svc.Sched drop

    sync input port faultIn: FaultRecordPort

    output port modeOut: ModeFlag

    output port modeToMonitor: ModeFlag

    output port clearOut: ClearNotify

    @ Advisor and human. The uplink filter is what blocks every other opcode.
    async command AGENT_PROPOSE(
      action: Mode
      wheel_index: U8
      confidence: F32
      sim_time_ns: U64
    ) opcode 0

    @ Human only. AgentUplinkFilter never forwards FDIR.CLEAR.
    async command FDIR_CLEAR opcode 1

    telemetry MODE: Mode id 0

    telemetry SEVERITY: Severity id 1

    telemetry ELIGIBLE: U8 id 2

    telemetry SOC: F64 id 3

    telemetry SOLAR_SCALE: F64 id 4

    telemetry WHEEL_SPEED: WheelF64 id 5

    telemetry WHEEL_STDDEV: WheelF64 id 6

    telemetry TEMP_C: F64 id 7

    telemetry SENSOR_VALID: bool id 8

    telemetry LAST_PROPOSAL: Mode id 9

    telemetry PROPOSAL_DISPOSITION: Disposition id 10

    telemetry SIM_TIME_NS: U64 id 11

    event ModeTransition(
      $from: Mode
      to: Mode
      simTimeNs: U64
    ) severity activity high id 0 format "Mode {} -> {} at {} ns"

    event HoldInvalid(
      simTimeNs: U64
    ) severity warning low id 1 format "HOLD_INVALID at {} ns"

    event ProposalScored(
      disposition: Disposition
      action: Mode
    ) severity activity low id 2 format "Proposal {} action {}"

    event TempWarn(
      tempC: F64
    ) severity activity low id 3 format "Temperature warning {} C"

    import Fw.Command

    import Fw.Event

    import Fw.Channel

    time get port timeGetOut

  }

}
