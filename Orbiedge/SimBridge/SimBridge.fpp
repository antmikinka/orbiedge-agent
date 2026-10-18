module Orbiedge {

  @ TCP client of Basilisk sensor JSON on 127.0.0.1:5600.
  @ TCP server for mode JSON on 127.0.0.1:5601.
  @ TCP server for advisor lines on 127.0.0.1:50001.
  @ Does not open 0.0.0.0 or the human GDS port 50000.
  active component SimBridge {

    async input port run: Svc.Sched drop

    sync input port modeIn: ModeRequest

    output port timeEpochSet: Svc.OsTimeEpoch

    output port powerOut: PowerState

    output port thermalOut: ThermalState

    output port adcsOut: AdcsState

    output port commitOut: SampleCommit

    output port sensorValidOut: SensorValid

    output port filterSetTime: SimTimeStamp

    output port proposalOut: Proposal

    event Listening(listenPort: U16) severity activity high id 0 format "SimBridge listening on {}"

    event BindFailed(listenPort: U16) severity warning high id 1 format "SimBridge bind failed on {}"

    event SensorLost severity warning high id 2 format "SimBridge sensor silent for 500 ms"

    import Fw.Event

    time get port timeGetOut

  }

}
