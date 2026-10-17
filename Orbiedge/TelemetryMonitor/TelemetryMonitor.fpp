module Orbiedge {

  @ Three F64 values, little-endian on the wire packet and native in telemetry.
  array F64x3 = [3] F64

  @ Four F64 values. Quaternion index 0 is the scalar.
  array F64x4 = [4] F64

  @ One decoded Basilisk sample.
  @ Quaternion arguments are scalar-first. busVoltage is derived from state of charge.
  port BasiliskState(
    simTimeNs: U64
    q0: F64
    q1: F64
    q2: F64
    q3: F64
    omegaX: F64
    omegaY: F64
    omegaZ: F64
    busVoltage: F64
    wheel1: F64
    wheel2: F64
    wheel3: F64
    faultFlags: U16
  )

  @ Ingests Basilisk state datagrams and publishes them as F Prime telemetry.
  @ Basilisk keeps the dynamics, sensors, actuators, and the stand-in attitude controller.
  @ This component does not command reaction wheels or thrusters.
  active component TelemetryMonitor {

    @ Connect a ZeroMQ SUB socket. The socket is used only on this component's thread.
    async command START(
      endpoint: string size 80
    ) opcode 0

    @ Close the ZeroMQ socket.
    async command STOP opcode 1

    @ Basilisk simulation time of the last accepted packet, nanoseconds.
    telemetry SimTimeNs: U64 id 0

    @ Body attitude quaternion, scalar first, converted from the Basilisk MRP.
    telemetry q_BN: F64x4 id 1

    @ Body angular rate relative to inertial, rad/s.
    telemetry omega_BN_B: F64x3 id 2

    @ Stand-in bus voltage derived from battery state of charge, volts.
    telemetry BusVoltage: F64 id 3

    @ Wheel speeds for RW1, RW2, and RW3, rad/s. RW3 is the body-Z wheel.
    telemetry WheelSpeeds: F64x3 id 4

    @ Scenario fault bits. Bit 0 is RW3 stiction, bit 1 is the power brownout, bit 2 is the IMU walk.
    telemetry FaultFlags: U16 id 5

    @ Packets that passed the size, magic, version, and CRC checks.
    telemetry PacketsAccepted: U32 id 6

    event Started(
      endpoint: string size 80
    ) severity activity high id 0 format "TelemetryMonitor subscribed at {}"

    event PacketAccepted(
      simTimeNs: U64
      faultFlags: U16
    ) severity activity low id 1 format "Accepted Basilisk state at {} ns, fault flags {}"

    event PacketRejected(
      reason: string size 80
    ) severity warning high id 2 format "Rejected Basilisk state: {}"

    event Stopped severity activity high id 3 format "TelemetryMonitor stopped"

    @ Queued so the ZeroMQ socket stays on this thread. A full queue drops the tick.
    @ The handler uses a zero receive timeout and returns without blocking.
    async input port schedIn: Svc.Sched drop

    output port stateOut: BasiliskState

    import Fw.Command
    import Fw.Event
    import Fw.Channel

    time get port timeGetOut

  }

}
