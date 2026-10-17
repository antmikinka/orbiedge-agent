"""LEO plant: one stand-in attitude controller, four HR16 wheels, JSON FDIR I/O.

The dynamics, gravity, hub, and initial orbit come from Basilisk's
``scenarioAttitudeFeedbackRWPower.py``. That example builds three orthogonal
Honeywell_HR16 wheels. This scenario adds a fourth, in a pyramid, because the
architecture jitter predicate treats the other three speeds as the quiet
channels.

The stand-in controller is still one chain: ``inertial3D``,
``attTrackingError``, ``mrpFeedback``, and ``rwMotorTorque``. A mode flag is
not a second MRP law. ``WheelTorqueGate`` sits between ``rwMotorTorque`` and
the wheel effector and edits that same command. There is no magnetorquer.
``MOMENTUM_DUMP`` is a stand-in: the gate scales the wheel torque commands by
0.2, toward a lower speed set-point, instead of dumping momentum with a
magnetic torque.

``rwMotorTorque.controlAxes_B`` on this Basilisk build is nine doubles (three
body control axes). Assigning the 3x4 pyramid matrix raises
``ValueError: Size mismatch. Expected 3*3 elements``. The identity is written
there so all three body axes stay controlled. The pyramid axes are the wheel
spin axes passed to ``rwFactory.create``. ``GS_ROW_MAJOR`` is that 3x4 matrix,
columns equal to the wheel axes, stored row-major. The factory publishes the
same axes wheel-major in ``GsMatrix_B``.

The RW power example has no solar panel. ``solar_scale`` stays scenario state.
While the solar family is armed, a second ``simplePowerSink`` (``solarDeficit``)
draws ``SOLAR_DEFICIT_AT_ZERO_W * (1 - solar_scale)`` and returns to 0 W once
state of charge is at or below 0.20, so a long run can pass through 0.30
without being driven through 0.15. Payload draw is a separate sink at -5 W,
and 0 W while ``LOAD_SHED``.

Published ``wheel_speed_rad_s`` is the sensed speed FDIR reads. ``rw_jitter``
holds the other wheels at their onset sample and replaces one wheel with a
+/- 3 rad/s square wave, which is the injection the stddev predicate scores.
The stand-in controller still reads ``rwSpeedOutMsg``. ``rw_saturation`` uses
``ExtForceTorque`` and publishes the effector speed. The body torque stays on
past 0.9 of the wheel limit and comes off only near the hard stop. Basilisk is
not a thermal simulator: ``temp_c`` and ``heater_on`` are integrated here.
"""

from __future__ import annotations

import argparse
import errno
import json
import select
import socket
import sys

try:
    import Basilisk  # noqa: F401
except ImportError as exc:
    raise SystemExit(
        "Basilisk cannot be imported. This plant was not executed. "
        f"Install Basilisk in this interpreter ({sys.executable}). {exc}"
    ) from exc

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback, rwMotorTorque
from Basilisk.simulation import (
    ReactionWheelPower,
    extForceTorque,
    reactionWheelStateEffector,
    simpleBattery,
    simpleNav,
    simplePowerSink,
    spacecraft,
)
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simHelpers,
    simIncludeGravBody,
    simIncludeRW,
)

STEP_S = 0.1
STEP_NS = macros.sec2nano(STEP_S)
NUM_WHEELS = 4
# Honeywell_HR16 Omega_max is 6000 RPM. 6000 * 2*pi/60.
WHEEL_MAX_RAD_S = 628.3185307179587
SATURATION_FRACTION = 0.9
# Body torque applied until a wheel would reach SATURATION_FRACTION of the HR16 limit.
SATURATION_TORQUE_B_NM = (0.0, 0.0, 0.05)
HR16_U_MAX_NM = 0.200
HR16_MAX_MOMENTUM_NMS = 50.0
DUMP_SCALE = 0.2
PAYLOAD_W = -5.0
STORAGE_CAPACITY_J = 300000.0
SOC_INIT = 0.8
SOLAR_SCALE_INIT = 1.0
SOLAR_SCALE_FLOOR = 0.4
SOLAR_RAMP_S = 30.0
TEMP_INIT_C = 20.0
TEMP_MARK_C = 60.0
# Ten seconds of heater time carries the scripted node through TEMP_MARK_C.
THERMAL_CROSS_S = 10.0
TEMP_RAMP_C_PER_S = (TEMP_MARK_C - TEMP_INIT_C) / THERMAL_CROSS_S
JITTER_AMPLITUDE_RAD_S = 3.0
JITTER_ONSET_S = 2.0
# Full deficit at solar_scale 0. Scaled by (1 - solar_scale). The power
# example has no array, so this sink is the degradation load.
SOLAR_DEFICIT_AT_ZERO_W = -2500.0
SOC_SOLAR_HOLD = 0.20
TEMP_HOLD_C = 70.0
ESCALATE_WINDOW_S = 30.0
ESCALATE_RISE_RAD_S2 = 1.0
REFERENCE_SIGMA_RN = (0.0, 0.0, 0.0)
INITIAL_SIGMA_BN = (0.1, 0.2, -0.3)
PUBLISH_ADDRESS = ("127.0.0.1", 5600)
MODE_ADDRESS = ("127.0.0.1", 5601)
MODES = ("NOMINAL", "ISOLATE_RW", "MOMENTUM_DUMP", "LOAD_SHED", "SAFE")
FAMILIES = ("nominal", "rw_saturation", "rw_jitter", "solar", "thermal", "escalate")

# Unit pyramid. The example's three orthogonal gsHat vectors are not used.
PYRAMID_AXES_B = (
    (0.57735026919, 0.57735026919, 0.57735026919),
    (-0.57735026919, 0.57735026919, 0.57735026919),
    (-0.57735026919, -0.57735026919, 0.57735026919),
    (0.57735026919, -0.57735026919, 0.57735026919),
)
# 3x4 row-major, columns are PYRAMID_AXES_B. Not writable to controlAxes_B.
GS_ROW_MAJOR = tuple(
    PYRAMID_AXES_B[column][row] for row in range(3) for column in range(4)
)
# Three body control axes, row-major identity. This is the legal controlAxes_B.
CONTROL_AXES_B = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

_CONNECT_IN_PROGRESS = {
    errno.EWOULDBLOCK,
    errno.EINPROGRESS,
    errno.EALREADY,
    10035,  # WSAEWOULDBLOCK
    10036,  # WSAEINPROGRESS
}


def _as_floats(value, count):
    flat = np.array(value, dtype=float).reshape(-1)
    if flat.size < count:
        raise RuntimeError(f"expected at least {count} values, got {flat.size}")
    return [float(item) for item in flat[:count]]


class WheelAvailability(sysModel.SysModel):
    """Tell rwMotorTorque which wheels are in the minimum-norm allocation.

    ``ISOLATE_RW`` marks the named wheel ``UNAVAILABLE`` so the stand-in
    allocator drops that column. Other modes leave all four available.
    """

    def __init__(self):
        super().__init__()
        self.ModelTag = "wheelAvailability"
        self.availOutMsg = messaging.RWAvailabilityMsg()
        self.mode = "NOMINAL"
        self.wheel_index = 0

    def _payload(self):
        payload = messaging.RWAvailabilityMsgPayload()
        flags = [messaging.AVAILABLE] * NUM_WHEELS
        if self.mode == "ISOLATE_RW" and 0 <= self.wheel_index < NUM_WHEELS:
            flags[self.wheel_index] = messaging.UNAVAILABLE
        payload.wheelAvailability = flags
        return payload

    def Reset(self, CurrentSimNanos):
        self.availOutMsg.write(self._payload(), CurrentSimNanos, self.moduleID)

    def UpdateState(self, CurrentSimNanos):
        self.availOutMsg.write(self._payload(), CurrentSimNanos, self.moduleID)


class WheelTorqueGate(sysModel.SysModel):
    """Edit the stand-in motor command before the wheel effector sees it.

    This is the only mode actuator. It does not compute a new MRP torque.
    ``MOMENTUM_DUMP`` multiplies the four wheel commands by ``DUMP_SCALE``
    (0.2). That scale is the set-point reduction used because this plant has
    no magnetorquer. ``ISOLATE_RW`` forces the named command to 0 here as
    well as through the availability message. ``SAFE`` forces every command
    to 0, which is the rate-command path being held off.
    """

    def __init__(self):
        super().__init__()
        self.ModelTag = "wheelTorqueGate"
        self.torqueInMsg = messaging.ArrayMotorTorqueMsgReader()
        self.torqueOutMsg = messaging.ArrayMotorTorqueMsg()
        self.mode = "NOMINAL"
        self.wheel_index = 0

    def Reset(self, CurrentSimNanos):
        self._write([0.0] * NUM_WHEELS, CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos):
        incoming = self.torqueInMsg()
        torque = np.array(incoming.motorTorque, dtype=float).reshape(-1).copy()
        if torque.size < NUM_WHEELS:
            padded = np.zeros(NUM_WHEELS, dtype=float)
            padded[: torque.size] = torque
            torque = padded
        if self.mode == "SAFE":
            torque[:] = 0.0
        elif self.mode == "MOMENTUM_DUMP":
            torque[:NUM_WHEELS] *= DUMP_SCALE
        elif self.mode == "ISOLATE_RW" and 0 <= self.wheel_index < NUM_WHEELS:
            torque[self.wheel_index] = 0.0
        self._write(torque, CurrentSimNanos)

    def _write(self, torque, current_sim_nanos):
        output = messaging.ArrayMotorTorqueMsgPayload()
        output.motorTorque = [float(item) for item in torque]
        self.torqueOutMsg.write(output, current_sim_nanos, self.moduleID)


class ModeClient:
    """Non-blocking TCP client for one JSON mode line per write on :5601."""

    def __init__(self, host, port):
        self._host = host
        self._port = port
        self._sock = None
        self._connecting = False
        self._buffer = ""

    def close(self):
        sock = self._sock
        self._sock = None
        self._connecting = False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                return

    def _open(self):
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(False)
        err = sock.connect_ex((self._host, self._port))
        if err == 0:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = sock
            self._connecting = False
            return
        if err in _CONNECT_IN_PROGRESS:
            self._sock = sock
            self._connecting = True
            return
        sock.close()

    def _finish_connect(self):
        if not self._connecting or self._sock is None:
            return
        _readable, writable, errored = select.select([self._sock], [self._sock], [self._sock], 0)
        if not writable and not errored:
            return
        err = self._sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err != 0:
            self.close()
            return
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._connecting = False

    def poll(self):
        """Return the newest complete mode object, or None if nothing arrived.

        Connection refused, a half-open connect, and an empty socket all leave
        the caller on its current mode. A dead peer is dropped and retried.
        """
        self._open()
        self._finish_connect()
        if self._sock is None or self._connecting:
            return None
        while True:
            try:
                chunk = self._sock.recv(4096)
            except BlockingIOError:
                break
            except OSError:
                self.close()
                break
            if not chunk:
                self.close()
                break
            self._buffer += chunk.decode("utf-8", errors="replace")
        latest = None
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict) and "mode" in payload:
                latest = payload
        return latest


class StatePublisher:
    """Bind 127.0.0.1:5600 and write one JSON line per accepted client."""

    def __init__(self, host, port):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((host, port))
        except OSError as exc:
            server.close()
            raise SystemExit(
                f"Could not bind the plant publisher to {host}:{port}. {exc}"
            ) from exc
        server.listen(1)
        server.setblocking(False)
        self._server = server
        self._client = None

    def _accept(self):
        if self._client is not None:
            return
        try:
            client, _addr = self._server.accept()
        except BlockingIOError:
            return
        client.setblocking(False)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._client = client

    def send(self, record):
        self._accept()
        if self._client is None:
            return
        data = (json.dumps(record, separators=(",", ":")) + "\n").encode("utf-8")
        try:
            self._client.sendall(data)
        except BlockingIOError:
            return
        except OSError:
            self._drop_client()

    def _drop_client(self):
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass
        self._client = None

    def close(self):
        self._drop_client()
        try:
            self._server.close()
        except OSError:
            return


def _latch_mode(previous, payload):
    if not isinstance(payload, dict):
        return previous
    mode = str(payload.get("mode", "")).strip().upper()
    if mode not in MODES:
        return previous
    wheel_index = previous["wheel_index"]
    if payload.get("wheel_index") is not None:
        wheel_index = int(payload["wheel_index"])
    return {"mode": mode, "wheel_index": wheel_index}


def _read_sigma(sc_object):
    state = sc_object.scStateOutMsg.read()
    return _as_floats(state.sigma_BN, 3)


def _read_wheel_speeds(wheels):
    message = wheels.rwSpeedOutMsg.read()
    return _as_floats(message.wheelSpeeds, NUM_WHEELS)


def _read_soc(battery):
    status = battery.batPowerOutMsg.read()
    capacity = float(status.storageCapacity)
    if capacity <= 0.0:
        return 0.0
    return float(status.storageLevel) / capacity


def _saturation_torque(truth_speeds, enabled):
    """Body torque held until a wheel is near the HR16 hard stop.

    FDIR scores a wheel above 0.9 of Omega_max. This guard stays above that
    line and removes the torque only when the next ``u_max`` step would reach
    0.98 of Omega_max. ``Js`` is ``maxMomentum / Omega_max``.
    """
    if not enabled:
        return (0.0, 0.0, 0.0)
    js = HR16_MAX_MOMENTUM_NMS / WHEEL_MAX_RAD_S
    peak = max(abs(speed) for speed in truth_speeds)
    # SATURATION_FRACTION is the FDIR line. The torque stays on past it.
    hard_line = 0.98 * WHEEL_MAX_RAD_S
    if hard_line <= SATURATION_FRACTION * WHEEL_MAX_RAD_S:
        raise RuntimeError("hard-stop guard must remain above the FDIR line")
    if peak + (HR16_U_MAX_NM / js) * STEP_S >= hard_line:
        return (0.0, 0.0, 0.0)
    return SATURATION_TORQUE_B_NM


def _apply_sensed_speeds(truth, family, faults_on, state, time_s, mode):
    """Copy effector speeds, then apply publisher-only fault edits."""
    sensed = [float(speed) for speed in truth]
    if not faults_on:
        return sensed
    if family == "rw_jitter" and time_s >= state["jitter_onset_s"]:
        if "quiet" not in state:
            state["quiet"] = list(sensed)
        elapsed_steps = int(round((time_s - state["jitter_onset_s"]) / STEP_S))
        sensed = list(state["quiet"])
        sign = 1.0 if elapsed_steps % 2 == 0 else -1.0
        amplitude = JITTER_AMPLITUDE_RAD_S * (1.0 + state["seed"] * 0.01)
        sensed[state["jitter_wheel"]] = sign * amplitude
        return sensed
    if family == "escalate" and mode == "MOMENTUM_DUMP" and state["t0_s"] is None:
        state["t0_s"] = time_s
        state["floor"] = max(abs(speed) for speed in sensed)
    if family == "escalate" and state["t0_s"] is not None:
        elapsed = time_s - state["t0_s"]
        if 0.0 <= elapsed <= ESCALATE_WINDOW_S:
            state["floor"] += ESCALATE_RISE_RAD_S2 * STEP_S
            index = max(range(NUM_WHEELS), key=lambda i: abs(sensed[i]))
            sign = 1.0 if sensed[index] >= 0.0 else -1.0
            sensed[index] = sign * max(abs(sensed[index]), state["floor"])
    return sensed


def build_simulation():
    """Hub, point-mass Earth, and LEO state from scenarioAttitudeFeedbackRWPower."""
    sc_sim = SimulationBaseClass.SimBaseClass()
    sc_sim.SetProgressBar(False)
    process = sc_sim.CreateNewProcess("simProcess")
    process.addTask(sc_sim.CreateNewTask("simTask", STEP_NS))
    task = "simTask"

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "bsk-Sat"
    inertia = [
        900.0, 0.0, 0.0,
        0.0, 800.0, 0.0,
        0.0, 0.0, 600.0,
    ]
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(inertia)

    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    grav_factory.addBodiesTo(sc_object)

    rw_factory = simIncludeRW.rwFactory()
    rw_model = messaging.BalancedWheels
    # Example Omegas are 100, 200, and 300 RPM. The fourth wheel is 400 RPM.
    omegas_rpm = (100.0, 200.0, 300.0, 400.0)
    for index, (axis, omega_rpm) in enumerate(zip(PYRAMID_AXES_B, omegas_rpm)):
        kwargs = {
            "maxMomentum": HR16_MAX_MOMENTUM_NMS,
            "Omega": omega_rpm,
            "RWModel": rw_model,
        }
        # The example offsets only its third wheel. Keep that offset.
        if index == 2:
            kwargs["rWB_B"] = [0.5, 0.5, 0.5]
        rw_factory.create("Honeywell_HR16", list(axis), **kwargs)
    num_rw = rw_factory.getNumOfDevices()
    if num_rw != NUM_WHEELS:
        raise RuntimeError(f"expected {NUM_WHEELS} reaction wheels, created {num_rw}")

    wheels = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_factory.addToSpacecraft(sc_object.ModelTag, wheels, sc_object)
    # Gate runs before the effector so a mode latched for this step is the
    # command integrated on this step. The motor command it forwards was
    # written at the end of the previous step, after navigation.
    gate = WheelTorqueGate()
    sc_sim.AddModelToTask(task, gate, 400)
    disturbance = extForceTorque.ExtForceTorque()
    disturbance.ModelTag = "rwSaturationTorque"
    disturbance.extTorquePntB_B = [[0.0], [0.0], [0.0]]
    sc_object.addDynamicEffector(disturbance)
    sc_sim.AddModelToTask(task, disturbance, 360)
    sc_sim.AddModelToTask(task, wheels, 320)
    sc_sim.AddModelToTask(task, sc_object, 280)

    elements = orbitalMotion.ClassicElements()
    elements.a = 10000000.0
    elements.e = 0.01
    elements.i = 33.3 * macros.D2R
    elements.Omega = 48.2 * macros.D2R
    elements.omega = 347.8 * macros.D2R
    elements.f = 85.3 * macros.D2R
    position, velocity = orbitalMotion.elem2rv(mu, elements)
    sc_object.hub.r_CN_NInit = position
    sc_object.hub.v_CN_NInit = velocity
    sc_object.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    sc_object.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    nav = simpleNav.SimpleNav()
    nav.ModelTag = "SimpleNavigation"
    nav.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    sc_sim.AddModelToTask(task, nav, 200)

    rw_power = []
    for index in range(num_rw):
        power = ReactionWheelPower.ReactionWheelPower()
        power.ModelTag = sc_object.ModelTag + "RWPower" + str(index)
        power.basePowerNeed = 5.0
        power.mechToElecEfficiency = 0.5
        power.rwStateInMsg.subscribeTo(wheels.rwOutMsgs[index])
        rw_power.append(power)
        sc_sim.AddModelToTask(task, power, 190)

    payload = simplePowerSink.SimplePowerSink()
    payload.ModelTag = "payload"
    payload.nodePowerOut = PAYLOAD_W
    sc_sim.AddModelToTask(task, payload, 180)

    solar_deficit = simplePowerSink.SimplePowerSink()
    solar_deficit.ModelTag = "solarDeficit"
    solar_deficit.nodePowerOut = 0.0
    sc_sim.AddModelToTask(task, solar_deficit, 175)

    battery = simpleBattery.SimpleBattery()
    battery.ModelTag = sc_object.ModelTag
    battery.storageCapacity = STORAGE_CAPACITY_J
    battery.storedCharge_Init = STORAGE_CAPACITY_J * SOC_INIT
    for power in rw_power:
        battery.addPowerNodeToModel(power.nodePowerOutMsg)
    battery.addPowerNodeToModel(payload.nodePowerOutMsg)
    battery.addPowerNodeToModel(solar_deficit.nodePowerOutMsg)
    sc_sim.AddModelToTask(task, battery, 170)

    availability = WheelAvailability()
    sc_sim.AddModelToTask(task, availability, 150)

    reference = inertial3D.inertial3D()
    reference.ModelTag = "inertial3D"
    reference.sigma_R0N = list(REFERENCE_SIGMA_RN)
    sc_sim.AddModelToTask(task, reference, 140)

    att_error = attTrackingError.attTrackingError()
    att_error.ModelTag = "attErrorInertial3D"
    att_error.attRefInMsg.subscribeTo(reference.attRefOutMsg)
    att_error.attNavInMsg.subscribeTo(nav.attOutMsg)
    sc_sim.AddModelToTask(task, att_error, 130)

    feedback = mrpFeedback.mrpFeedback()
    feedback.ModelTag = "mrpFeedback"
    vehicle = messaging.VehicleConfigMsgPayload(ISCPntB_B=inertia)
    vehicle_msg = messaging.VehicleConfigMsg().write(vehicle)
    rw_config = rw_factory.getConfigMessage()
    feedback.guidInMsg.subscribeTo(att_error.attGuidOutMsg)
    feedback.vehConfigInMsg.subscribeTo(vehicle_msg)
    feedback.rwParamsInMsg.subscribeTo(rw_config)
    feedback.rwSpeedsInMsg.subscribeTo(wheels.rwSpeedOutMsg)
    feedback.K = 3.5
    feedback.Ki = -1
    feedback.P = 30.0
    feedback.integralLimit = 2.0 / feedback.Ki * 0.1
    sc_sim.AddModelToTask(task, feedback, 120)

    motor = rwMotorTorque.rwMotorTorque()
    motor.ModelTag = "rwMotorTorque"
    motor.controlAxes_B = list(CONTROL_AXES_B)
    motor.vehControlInMsg.subscribeTo(feedback.cmdTorqueOutMsg)
    motor.rwParamsInMsg.subscribeTo(rw_config)
    motor.rwAvailInMsg.subscribeTo(availability.availOutMsg)
    sc_sim.AddModelToTask(task, motor, 110)
    gate.torqueInMsg.subscribeTo(motor.rwMotorTorqueOutMsg)
    wheels.rwMotorCmdInMsg.subscribeTo(gate.torqueOutMsg)

    sc_sim.kept_alive = {
        "grav_factory": grav_factory,
        "earth": earth,
        "rw_factory": rw_factory,
        "vehicle": vehicle_msg,
        "rw_config": rw_config,
        "gs_row_major": GS_ROW_MAJOR,
    }
    return {
        "sim": sc_sim,
        "sc": sc_object,
        "wheels": wheels,
        "battery": battery,
        "payload": payload,
        "solar_deficit": solar_deficit,
        "reference": reference,
        "gate": gate,
        "availability": availability,
        "motor": motor,
        "disturbance": disturbance,
    }


def _apply_latched_mode(models, latched):
    mode = latched["mode"]
    wheel_index = latched["wheel_index"]
    models["gate"].mode = mode
    models["gate"].wheel_index = wheel_index
    models["availability"].mode = mode
    models["availability"].wheel_index = wheel_index
    if mode == "LOAD_SHED":
        models["payload"].nodePowerOut = 0.0
    else:
        models["payload"].nodePowerOut = PAYLOAD_W
    if mode == "SAFE":
        # Hold the attitude the body already has. Wheel torque is zeroed in
        # the gate, so the rate error cannot command a further slew.
        try:
            held = _read_sigma(models["sc"])
        except Exception:
            held = list(INITIAL_SIGMA_BN)
        models["reference"].sigma_R0N = held
    else:
        models["reference"].sigma_R0N = list(REFERENCE_SIGMA_RN)


def _set_disturbance(disturbance, torque_b):
    disturbance.extTorquePntB_B = [[float(torque_b[0])], [float(torque_b[1])], [float(torque_b[2])]]


def run(duration_s, family, seed, use_tcp, faults_on):
    if duration_s <= 0.0:
        raise SystemExit("--duration must be positive")
    if family not in FAMILIES:
        raise SystemExit(f"--family must be one of {', '.join(FAMILIES)}")
    steps = int(round((duration_s * 1.0e9) / STEP_NS))
    if steps < 1:
        raise SystemExit("--duration is shorter than one 0.1 s step")

    models = build_simulation()
    sc_sim = models["sim"]
    publisher = StatePublisher(*PUBLISH_ADDRESS) if use_tcp else None
    modes = ModeClient(*MODE_ADDRESS) if use_tcp else None
    sc_sim.InitializeSimulation()
    blank = messaging.ArrayMotorTorqueMsgPayload()
    blank.motorTorque = [0.0] * NUM_WHEELS
    models["motor"].rwMotorTorqueOutMsg.write(blank)

    latched = {"mode": "NOMINAL", "wheel_index": 0}
    solar_scale = SOLAR_SCALE_INIT
    temp_c = TEMP_INIT_C
    fault_state = {
        "t0_s": None,
        "floor": 0.0,
        "jitter_onset_s": JITTER_ONSET_S,
        "jitter_wheel": seed % NUM_WHEELS,
        "seed": seed,
    }
    truth = _read_wheel_speeds(models["wheels"])
    stop_ns = 0
    try:
        for _step in range(steps):
            _apply_latched_mode(models, latched)
            mode = latched["mode"]
            heater_on = faults_on and family == "thermal" and mode != "SAFE"
            if faults_on and family == "solar":
                solar_step = (SOLAR_SCALE_INIT - SOLAR_SCALE_FLOOR) * STEP_S / SOLAR_RAMP_S
                solar_scale = max(SOLAR_SCALE_FLOOR, solar_scale - solar_step)
                soc_now = _read_soc(models["battery"])
                if soc_now > SOC_SOLAR_HOLD:
                    deficit = SOLAR_DEFICIT_AT_ZERO_W * (1.0 - solar_scale)
                else:
                    deficit = 0.0
            else:
                deficit = 0.0
            models["solar_deficit"].nodePowerOut = deficit
            torque = _saturation_torque(
                truth,
                faults_on and family == "rw_saturation",
            )
            _set_disturbance(models["disturbance"], torque)
            stop_ns += STEP_NS
            sc_sim.ConfigureStopTime(stop_ns)
            sc_sim.ExecuteSimulation()

            truth = _read_wheel_speeds(models["wheels"])
            time_s = stop_ns * 1.0e-9
            if heater_on and temp_c < TEMP_HOLD_C:
                temp_c = min(TEMP_HOLD_C, temp_c + TEMP_RAMP_C_PER_S * STEP_S)
            sensed = _apply_sensed_speeds(
                truth, family, faults_on, fault_state, time_s, mode
            )
            record = {
                "sim_time_ns": int(stop_ns),
                "soc": _read_soc(models["battery"]),
                "solar_scale": solar_scale,
                "wheel_speed_rad_s": sensed,
                "temp_c": temp_c,
                "heater_on": heater_on,
            }
            print(json.dumps(record))
            if publisher is not None:
                publisher.send(record)
            if modes is not None:
                incoming = modes.poll()
                if incoming is not None:
                    latched = _latch_mode(latched, incoming)
    finally:
        if publisher is not None:
            publisher.close()
        if modes is not None:
            modes.close()
    where = f"publisher {PUBLISH_ADDRESS[0]}:{PUBLISH_ADDRESS[1]}" if use_tcp else "without TCP"
    print(
        f"completed {steps} steps at {STEP_S:.1f} s, family={family}, "
        f"faults={'on' if faults_on else 'off'}, {where}"
    )
    return steps


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="LEO RW power plant with one stand-in controller and FDIR JSON I/O."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="simulation length in seconds (default 1 s, a short probe)",
    )
    parser.add_argument("--family", choices=FAMILIES, default="nominal")
    parser.add_argument("--seed", type=int, default=1, help="NumPy seed for rw_jitter")
    parser.add_argument(
        "--no-tcp",
        action="store_true",
        help="run the dynamics without the :5600 publisher or the :5601 mode client",
    )
    parser.add_argument(
        "--no-faults",
        action="store_true",
        help="ignore --family and publish the unfaulted plant",
    )
    args = parser.parse_args(argv)
    run(
        duration_s=args.duration,
        family=args.family,
        seed=args.seed,
        use_tcp=not args.no_tcp,
        faults_on=not args.no_faults,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
