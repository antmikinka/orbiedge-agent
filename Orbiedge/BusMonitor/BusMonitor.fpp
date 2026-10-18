module Orbiedge {

  @ Passive predicate monitor. It records faults. It does not command actuators.
  @ Demo limits, not vehicle limits: wheel 650 rad/s, warn at 0.9 of that,
  @ SOC 0.30 / 0.15, temperature 40 C / 60 C.
  passive component BusMonitor {

    sync input port powerIn: PowerState

    sync input port thermalIn: ThermalState

    sync input port adcsIn: AdcsState

    sync input port commitIn: SampleCommit

    sync input port currentMode: ModeRequest

    sync input port clearSticky: ClearSticky

    output port faultOut: FaultRecord

    telemetry Severity: Severity id 0

    telemetry MaxWheel: F64 id 1

    event Predicate(
      faultSeverity: Severity
      modeDefault: Mode
      wheelIndex: U8
    ) severity activity high id 0 format "BusMonitor {} default {} wheel {}"

    import Fw.Event

    import Fw.Channel

    time get port timeGetOut

  }

}
