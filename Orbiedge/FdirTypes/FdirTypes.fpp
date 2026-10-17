# Not on the configure path. BusMonitor.fpp already defines these names
# in module Orbiedge, and several fields differ.
module Orbiedge {

  @ Flight mode. FdirManager is the only component that may select one.
  enum Mode : U8 {
    NOMINAL
    ISOLATE_RW
    MOMENTUM_DUMP
    LOAD_SHED
    SAFE
  }

  enum Severity : U8 {
    NONE
    WARNING
    CRITICAL
  }

  @ What FdirManager did with the latest advisor proposal.
  enum Disposition : U8 {
    NONE
    AGREE
    DISAGREE
    LATE
    REJECT
    HOLD_INVALID
  }

  port PowerState(simTimeNs: U64, soc: F64, solarScale: F64)

  port ThermalState(simTimeNs: U64, tempC: F64, heaterOn: bool)

  port AdcsState(
    simTimeNs: U64
    wheel0: F64
    wheel1: F64
    wheel2: F64
    wheel3: F64
  )

  @ One committed sensor sample. Stores do not publish a fault until this fires.
  port SampleCommit(simTimeNs: U64)

  port FaultRecord(
    simTimeNs: U64
    faultSeverity: Severity
    modeDefault: Mode
    wheelIndex: U8
    eligibleCount: U8
    tempC: F64
    soc: F64
    maxWheel: F64
  )

  port ModeRequest(simTimeNs: U64, mode: Mode, wheelIndex: U8)

  @ opcode 0 is AGENT.PROPOSE. Any other opcode is refused by AgentUplinkFilter.
  port Proposal(
    opcode: U32
    action: Mode
    wheelIndex: U8
    confidence: F32
    simTimeNs: U64
  )

  port SensorValid(valid: bool)

  port SimTimeStamp(simTimeNs: U64)

  @ Human clear. The advisor path has no connection to this port.
  port ClearSticky(source: U8)

}
