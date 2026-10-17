module Orbiedge {

  @ Flight mode. ISOLATE_RW and SAFE stay until the human command FDIR.CLEAR.
  enum Mode : U8 {
    NOMINAL = 0
    ISOLATE_RW = 1
    MOMENTUM_DUMP = 2
    LOAD_SHED = 3
    SAFE = 4
  }

  @ NONE, WARNING, or CRITICAL. Temperature in the warning band is NONE.
  enum Severity : U8 {
    NONE = 0
    WARNING = 1
    CRITICAL = 2
  }

  @ Advisor score for one consumed proposal. HOLD_INVALID is a sensor-loss hold.
  enum Disposition : U8 {
    NONE = 0
    AGREE = 1
    DISAGREE = 2
    LATE = 3
    REJECT = 4
    HOLD_INVALID = 5
  }

  @ Eligible bits match Mode values: bit 0 NOMINAL through bit 4 SAFE.
  port FaultRecordPort(
    simTimeNs: U64
    eligible: U8
    defaultMode: Mode
    severity: Severity
    wheelIndex: U8
    sensorValid: bool
    quiet: bool
    tempWarn: bool
    soc: F64
    solarScale: F64
    tempC: F64
    wheel0: F64
    wheel1: F64
    wheel2: F64
    wheel3: F64
    std0: F64
    std1: F64
    std2: F64
    std3: F64
  )

  port PowerStatePort(
    simTimeNs: U64
    soc: F64
    solarScale: F64
    valid: bool
  )

  port ThermalStatePort(
    simTimeNs: U64
    tempC: F64
    heaterOn: bool
    valid: bool
  )

  port AdcsStatePort(
    simTimeNs: U64
    wheel0: F64
    wheel1: F64
    wheel2: F64
    wheel3: F64
    valid: bool
  )

  port ModeFlag(
    simTimeNs: U64
    mode: Mode
    wheelIndex: U8
  )

  port Proposal(
    opcode: string size 80
    action: Mode
    wheelIndex: U8
    confidence: F32
    simTimeNs: U64
    nowSimTimeNs: U64
    wallTimeNs: U64
  )

  port ClearNotify(
    reset: bool
  )

  @ Compares and counts. No sockets. Runs on the SimBridge thread.
  @ PowerState, then ThermalState, then AdcsState form one sample.
  passive component BusMonitor {

    sync input port powerIn: PowerStatePort

    sync input port thermalIn: ThermalStatePort

    sync input port adcsIn: AdcsStatePort

    @ Last commanded mode, so dump-rise can see MOMENTUM_DUMP.
    sync input port modeIn: ModeFlag

    sync input port clearIn: ClearNotify

    @ Port FaultRecord. Handlers only compare and count.
    output port FaultRecord: FaultRecordPort

    param WHEEL_MAX_RAD_S: F64 default 628.3185307179587

    param OVERSPEED_FRACTION: F64 default 0.9

    param OVERSPEED_SAMPLES: U32 default 10

    param DUMP_RISE_NS: U64 default 30000000000

    param STDDEV_HIGH: F64 default 1.0

    param STDDEV_QUIET: F64 default 0.2

    param STDDEV_SAMPLES: U32 default 20

    param SOC_WARN: F64 default 0.30

    param SOC_CRIT: F64 default 0.15

    param SOC_SAMPLES: U32 default 5

    param TEMP_WARN_C: F64 default 40.0

    param TEMP_CRIT_C: F64 default 60.0

    param TEMP_CRIT_SAMPLES: U32 default 50

    param TEMP_WARN_SAMPLES: U32 default 100

    param QUIET_DWELL_NS: U64 default 60000000000

    param get port prmGetOut

    param set port prmSetOut

    time get port timeGetOut

  }

}
