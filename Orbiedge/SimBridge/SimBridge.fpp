module Orbiedge {

  @ Active bridge. schedIn is async drop so the rate group only enqueues.
  @ The handler runs on this component's thread. That thread owns the TCP client
  @ to 127.0.0.1:5600 and the mode writer bound to 127.0.0.1:5601.
  @ Svc.Time (Svc/Interfaces/Time.fpp) has only timeGetPort. No set port was found.
  @ Svc.OsTime setEpoch and SetCurrentTime are a different component. stampSimTime(U64)
  @ is the typed hook. The topology must connect the real time-set port later.
  active component SimBridge {

    async input port schedIn: Svc.Sched drop

    @ Mode from FdirManager. Queued so the JSON write stays on this thread.
    async input port modeIn: ModeFlag

    output port PowerState: PowerStatePort

    output port ThermalState: ThermalStatePort

    output port AdcsState: AdcsStatePort

    telemetry SENSOR_VALID: bool id 0

    event SensorLoss(
      simTimeNs: U64
    ) severity warning high id 0 format "Sensor loss at {} ns; last sample invalid"

    event ParseFailed severity warning low id 1 format "Sensor JSON line rejected" throttle 5

    event LinkFailed severity warning low id 2 format "127.0.0.1 sensor or mode socket failed" throttle 5

    import Fw.Event

    import Fw.Channel

    time get port timeGetOut

  }

}
