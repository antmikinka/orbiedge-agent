"""3U CubeSat in low Earth orbit with a stand-in reaction-wheel controller.

Basilisk is the truth model. It owns the dynamics, the environment, the
sensors, the actuators, and the nominal pointing law. The F Prime
TelemetryMonitor only receives the serialized state packet. It does not
write wheel torque.

Environment, using stock Basilisk modules:

- Gravity: GGM03S degree 2 (J2). The scenario stops if that file is missing.
- Drag: NRLMSISE-00 density from MsisAtmosphere, cannonball DragDynamicEffector.
- Solar radiation pressure: two rigid panel facets on FacetSRPDynamicEffector.
- Magnetic field: World Magnetic Model (WMM2025). Basilisk has no IGRF class.
  WMM is the operational Earth field model this script uses.

The stand-in controller is inertial3D, attTrackingError, mrpFeedback, and
rwMotorTorque. simpleNav feeds that loop, so the injected IMU walk does not
steer the vehicle. The exported gyro is the IMU product, after senRotBias.

Faults are physical degradations in this script:

- 120 s: RW3 (body +Z, wheel index 2) command is clipped and a friction
  spike opposes the wheel speed. A Python SysModel sits between
  rwMotorTorque and the wheel effector.
- 240 s: both solar-panel efficiencies drop to 40 percent of nominal.
- 360 s: a random walk is written into ImuSensor.senRotBias before each
  step. That member is added on every IMU update.

Bus voltage in the packet is derived from battery state of charge.
SimpleBattery does not publish a voltage.

The task period is 0.5 s. After each ExecuteSimulation chunk the script
reads the sysModel outputs and publishes one datagram on ZeroMQ topic
``bsk.state``. A Basilisk 2.12 install does not wrap ``setUseApHistory`` or
``setPlanetPolarRadius``. Those calls are skipped unless the binding has them.
The constructor already sets Earth's equatorial radius and the daily-Ap switch.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
import time

import numpy as np

if importlib.util.find_spec("Basilisk") is None:
    raise SystemExit(
        'Basilisk is not installed. pip install "bsk[all]" and pyzmq. '
        "This scenario was not executed."
    )

from Basilisk.architecture import messaging, sysModel
from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback, rwMotorTorque
from Basilisk.simulation import (
    coarseSunSensor,
    dragDynamicEffector,
    eclipse,
    facetSRPDynamicEffector,
    imuSensor,
    magneticFieldWMM,
    msisAtmosphere,
    reactionWheelStateEffector,
    simpleBattery,
    simpleNav,
    simplePowerSink,
    simpleSolarPanel,
    spacecraft,
    starTracker,
    thrusterDynamicEffector,
)
from Basilisk.utilities import (
    RigidBodyKinematics,
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simHelpers,
    simIncludeGravBody,
    simIncludeRW,
    simIncludeThruster,
)
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path
from Basilisk.architecture import astroConstants

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim.bridge.packet import (
    FAULT_BROWNOUT,
    FAULT_IMU_WALK,
    FAULT_RW3_STICTION,
    TOPIC,
    derived_bus_voltage,
    pack_state,
)

EPOCH = "2026 JAN 01 00:00:00.000 (UTC)"
STEP_S = 0.5
STEP_NS = 500_000_000
STICTION_NS = 120 * 1_000_000_000
BROWNOUT_NS = 240 * 1_000_000_000
IMU_WALK_NS = 360 * 1_000_000_000

# Bus 3.6 kg plus two 0.20 kg panels. Three BCT RWP015 wheels are 0.130 kg
# each and are included here because BalancedWheels does not add wheel mass.
HUB_MASS_KG = 4.39
# Flat row-major inertia. np2EigenMatrix3d indexes nine elements, not a 3x3 array.
HUB_INERTIA = [
    0.050, 0.0, 0.0,
    0.0, 0.032, 0.0,
    0.0, 0.0, 0.025,
]
PANEL_AREA_M2 = 0.03
PANEL_EFFICIENCY = 0.28
PANEL_COP_M = 0.20
LOAD_W = 5.0
BATTERY_J = 40.0 * 3600.0
RW3_INDEX = 2
RW3_CLIP_NM = 2.0e-4
RW3_FRICTION_NM = 1.5e-4
IMU_WALK_SIGMA = 1.0e-4  # rad/s/sqrt(s)
IMU_BIAS_LIMIT = 1.0e-2  # rad/s, numerical guard


def support_file(data_file):
    """Resolve a Basilisk support-data file or stop.

    A missing J2 coefficient file or WMM coefficient file is a failed
    scenario. The script does not silently switch to point-mass gravity
    or a dipole field.
    """
    try:
        path = str(get_path(data_file))
    except Exception as exc:
        raise SystemExit(
            "Could not resolve Basilisk support data "
            f"{data_file}. J2 and WMM stay required.\n{exc}"
        ) from exc
    if not os.path.isfile(path):
        raise SystemExit(f"Basilisk support file is missing: {path}")
    return path


def as_floats(value, count):
    flat = np.array(value, dtype=float).reshape(-1)
    if flat.size < count:
        raise RuntimeError(f"expected at least {count} values, got {flat.size}")
    return [float(item) for item in flat[:count]]


class Rw3Stiction(sysModel.SysModel):
    """Clip RW3 torque and add a friction spike opposing wheel speed.

    The module is a director between rwMotorTorque and the reaction-wheel
    effector. While stiction is off it copies the command through.
    """

    def __init__(self, *args):
        super().__init__(*args)
        self.ModelTag = "rw3Stiction"
        self.torqueInMsg = messaging.ArrayMotorTorqueMsgReader()
        self.speedInMsg = messaging.RWSpeedMsgReader()
        self.torqueOutMsg = messaging.ArrayMotorTorqueMsg()
        self.stiction_on = False

    def Reset(self, CurrentSimNanos):
        return

    def UpdateState(self, CurrentSimNanos):
        torque_payload = self.torqueInMsg()
        speed_payload = self.speedInMsg()
        torques = as_floats(torque_payload.motorTorque, RW3_INDEX + 1)
        # Preserve the full command width the effector expects.
        full = list(np.array(torque_payload.motorTorque, dtype=float).reshape(-1))
        speeds = as_floats(speed_payload.wheelSpeeds, RW3_INDEX + 1)
        if self.stiction_on:
            command = torques[RW3_INDEX]
            command = min(RW3_CLIP_NM, max(-RW3_CLIP_NM, command))
            wheel_speed = speeds[RW3_INDEX]
            if wheel_speed > 0.0:
                command -= RW3_FRICTION_NM
            elif wheel_speed < 0.0:
                command += RW3_FRICTION_NM
            full[RW3_INDEX] = command
        output = messaging.ArrayMotorTorqueMsgPayload()
        output.motorTorque = full
        self.torqueOutMsg.write(output, CurrentSimNanos, self.moduleID)


def quiet_space_weather(atmo):
    """Connect the 23 NRLMSISE-00 space-weather inputs at quiet levels.

    Index order matches msisAtmosphere: 0 is daily Ap, 1..20 are the
    three-hour Ap history, 21 and 22 are the two F10.7 values.
    """
    ap = 10.0
    f107 = 140.0
    values = [ap] * 21 + [f107, f107]
    messages = []
    for index, value in enumerate(values):
        payload = messaging.SwDataMsgPayload()
        payload.dataValue = value
        message = messaging.SwDataMsg().write(payload)
        messages.append(message)
        atmo.swDataInMsgs[index].subscribeTo(message)
    return messages


def add_reaction_wheels(task_name, sc_sim, sc_object):
    factory = simIncludeRW.rwFactory()
    axes = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
    for axis in axes:
        # BCT_RWP015 replaces a caller-supplied maxMomentum. Do not pass Js.
        factory.create(
            "BCT_RWP015",
            list(axis),
            Omega=100.0,
            rWB_B=[0.0, 0.0, 0.0],
            RWModel=messaging.BalancedWheels,
        )
    effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    factory.addToSpacecraft("ReactionWheels", effector, sc_object)
    sc_sim.AddModelToTask(task_name, effector, 250)
    return factory, effector


def add_thrusters(task_name, sc_sim, sc_object):
    factory = simIncludeThruster.thrusterFactory()
    locations = (
        [0.04, 0.04, -0.15],
        [-0.04, 0.04, -0.15],
        [-0.04, -0.04, -0.15],
        [0.04, -0.04, -0.15],
    )
    for location in locations:
        factory.create("MOOG_Monarc_1", location, [0.0, 0.0, -1.0])
    effector = thrusterDynamicEffector.ThrusterDynamicEffector()
    factory.addToSpacecraft("acsThrusters", effector, sc_object)
    command = messaging.THRArrayOnTimeCmdMsgPayload()
    command.OnTimeRequest = [0.0, 0.0, 0.0, 0.0]
    command_msg = messaging.THRArrayOnTimeCmdMsg().write(command)
    effector.cmdsInMsg.subscribeTo(command_msg)
    sc_sim.AddModelToTask(task_name, effector, 250)
    return effector, command_msg


def build_simulation():
    sc_sim = SimulationBaseClass.SimBaseClass()
    process = sc_sim.CreateNewProcess("simProcess")
    process.addTask(sc_sim.CreateNewTask("simTask", macros.sec2nano(STEP_S)))
    task = "simTask"

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "cubesat3u"
    sc_object.hub.mHub = HUB_MASS_KG
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(HUB_INERTIA)
    vehicle = messaging.VehicleConfigMsgPayload()
    vehicle.ISCPntB_B = list(HUB_INERTIA)
    vehicle_msg = messaging.VehicleConfigMsg().write(vehicle)

    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True
    sun = grav_factory.createSun()
    j2_path = support_file(DataFile.LocalGravData.GGM03S_J2_only)
    earth.useSphericalHarmonicsGravityModel(j2_path, 2)
    grav_factory.addBodiesTo(sc_object)

    spice = grav_factory.createSpiceInterface(time=EPOCH)
    if not spice.SPICELoaded:
        raise SystemExit("SPICE kernels did not load. Sun and eclipse geometry would be wrong.")
    sc_sim.AddModelToTask(task, spice, 400)
    earth_msg = spice.planetStateOutMsgs[0]
    sun_msg = spice.planetStateOutMsgs[1]

    epoch_msg = simHelpers.timeStringToGregorianUTCMsg(EPOCH)

    atmo = msisAtmosphere.MsisAtmosphere()
    atmo.ModelTag = "msis"
    # The constructor already sets the equatorial radius MSIS requires.
    # The polar-radius setter is not wrapped in every Basilisk build.
    atmo.planetRadius = orbitalMotion.REQ_EARTH * 1000.0
    if hasattr(atmo, "setPlanetPolarRadius"):
        atmo.setPlanetPolarRadius(orbitalMotion.RP_EARTH * 1000.0)
    # 2.12 leaves the daily-Ap switch on by default. Later builds expose the setter.
    if hasattr(atmo, "setUseApHistory"):
        atmo.setUseApHistory(False)
    atmo.addSpacecraftToModel(sc_object.scStateOutMsg)
    atmo.epochInMsg.subscribeTo(epoch_msg)
    space_weather = quiet_space_weather(atmo)
    sc_sim.AddModelToTask(task, atmo, 350)

    mag = magneticFieldWMM.MagneticFieldWMM()
    mag.ModelTag = "WMM"
    mag.configureWMMFile(support_file(DataFile.MagneticFieldData.WMM))
    mag.addSpacecraftToModel(sc_object.scStateOutMsg)
    mag.epochInMsg.subscribeTo(epoch_msg)
    sc_sim.AddModelToTask(task, mag, 350)

    eclipse_model = eclipse.Eclipse()
    eclipse_model.addSpacecraftToModel(sc_object.scStateOutMsg)
    eclipse_model.addPlanetToModel(earth_msg)
    eclipse_model.sunInMsg.subscribeTo(sun_msg)
    sc_sim.AddModelToTask(task, eclipse_model, 350)
    eclipse_msg = eclipse_model.eclipseOutMsgs[0]

    drag = dragDynamicEffector.DragDynamicEffector()
    drag.ModelTag = "cannonballDrag"
    drag.coreParams.projectedArea = PANEL_AREA_M2
    drag.coreParams.dragCoeff = 2.2
    drag.coreParams.comOffset = [0.0, 0.0, 0.0]
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])
    sc_object.addDynamicEffector(drag)
    sc_sim.AddModelToTask(task, drag, 340)

    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    srp = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    srp.ModelTag = "panelSrp"
    srp.setNumFacets(2)
    srp.addFacet(PANEL_AREA_M2, identity, [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, PANEL_COP_M, 0.0], 0.1, 0.8)
    srp.addFacet(PANEL_AREA_M2, identity, [0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, -PANEL_COP_M, 0.0], 0.1, 0.8)
    srp.sunInMsg.subscribeTo(sun_msg)
    srp.sunEclipseInMsg.subscribeTo(eclipse_msg)
    sc_object.addDynamicEffector(srp)
    sc_sim.AddModelToTask(task, srp, 340)

    rw_factory, wheels = add_reaction_wheels(task, sc_sim, sc_object)
    _thrusters, thruster_cmd = add_thrusters(task, sc_sim, sc_object)
    sc_sim.AddModelToTask(task, sc_object, 200)

    elements = orbitalMotion.ClassicElements()
    elements.a = astroConstants.REQ_EARTH * 1.0e3 + 400.0e3
    elements.e = 0.0
    elements.i = 51.6 * macros.D2R
    elements.Omega = 0.0
    elements.omega = 0.0
    elements.f = 0.0
    position, velocity = orbitalMotion.elem2rv(earth.mu, elements)
    sc_object.hub.r_CN_NInit = position
    sc_object.hub.v_CN_NInit = velocity
    sc_object.hub.sigma_BNInit = [[0.05], [-0.03], [0.02]]
    sc_object.hub.omega_BN_BInit = [[0.002], [-0.001], [0.001]]

    nav = simpleNav.SimpleNav()
    nav.ModelTag = "SimpleNavigation"
    nav.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    sc_sim.AddModelToTask(task, nav, 150)

    imu = imuSensor.ImuSensor()
    imu.ModelTag = "imu"
    imu.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    imu.setBodyToPlatformDCM(0.0, 0.0, 0.0)
    imu.PMatrixGyro = [[1.0e-6, 0.0, 0.0], [0.0, 1.0e-6, 0.0], [0.0, 0.0, 1.0e-6]]
    imu.senRotBias = [0.0, 0.0, 0.0]
    sc_sim.AddModelToTask(task, imu, 150)

    tracker = starTracker.StarTracker()
    tracker.ModelTag = "starTracker"
    tracker.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    tracker.dcm_CB = identity
    tracker.PMatrix = [[1.0e-4, 0.0, 0.0], [0.0, 1.0e-4, 0.0], [0.0, 0.0, 1.0e-4]]
    sc_sim.AddModelToTask(task, tracker, 150)

    css_axes = ([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0])
    css_sensors = []
    for index, normal in enumerate(css_axes):
        sensor = coarseSunSensor.CoarseSunSensor()
        sensor.ModelTag = f"css{index}"
        sensor.nHat_B = list(normal)
        sensor.sunInMsg.subscribeTo(sun_msg)
        sensor.stateInMsg.subscribeTo(sc_object.scStateOutMsg)
        sensor.sunEclipseInMsg.subscribeTo(eclipse_msg)
        css_sensors.append(sensor)
        sc_sim.AddModelToTask(task, sensor, 150)

    panels = []
    for name, normal in (("panelPlusY", [0.0, 1.0, 0.0]), ("panelMinusY", [0.0, -1.0, 0.0])):
        panel = simpleSolarPanel.SimpleSolarPanel()
        panel.ModelTag = name
        panel.setPanelParameters(normal, PANEL_AREA_M2, PANEL_EFFICIENCY)
        panel.stateInMsg.subscribeTo(sc_object.scStateOutMsg)
        panel.sunInMsg.subscribeTo(sun_msg)
        panel.sunEclipseInMsg.subscribeTo(eclipse_msg)
        panels.append(panel)
        sc_sim.AddModelToTask(task, panel, 300)

    sink = simplePowerSink.SimplePowerSink()
    sink.ModelTag = "busLoad"
    sink.nodePowerOut = -LOAD_W
    sc_sim.AddModelToTask(task, sink, 300)

    battery = simpleBattery.SimpleBattery()
    battery.ModelTag = "battery"
    battery.storageCapacity = BATTERY_J
    battery.storedCharge_Init = 0.9 * BATTERY_J
    for panel in panels:
        battery.addPowerNodeToModel(panel.nodePowerOutMsg)
    battery.addPowerNodeToModel(sink.nodePowerOutMsg)
    sc_sim.AddModelToTask(task, battery, 280)

    reference = inertial3D.inertial3D()
    reference.ModelTag = "inertial3D"
    reference.sigma_R0N = [0.0, 0.0, 0.0]
    sc_sim.AddModelToTask(task, reference, 130)

    tracker_error = attTrackingError.attTrackingError()
    tracker_error.ModelTag = "attTrackingError"
    tracker_error.attRefInMsg.subscribeTo(reference.attRefOutMsg)
    tracker_error.attNavInMsg.subscribeTo(nav.attOutMsg)
    sc_sim.AddModelToTask(task, tracker_error, 120)

    feedback = mrpFeedback.mrpFeedback()
    feedback.ModelTag = "mrpFeedback"
    # The stock example gains belong to a large inertia. These are scaled
    # for the 0.03-0.05 kg m^2 3U so the 4 mNm wheel limit is not saturated
    # by the initial 0.05 MRP and 0.002 rad/s rate.
    feedback.K = 0.008
    feedback.P = 0.08
    feedback.Ki = -1.0
    rw_config = rw_factory.getConfigMessage()
    feedback.guidInMsg.subscribeTo(tracker_error.attGuidOutMsg)
    feedback.vehConfigInMsg.subscribeTo(vehicle_msg)
    feedback.rwParamsInMsg.subscribeTo(rw_config)
    feedback.rwSpeedsInMsg.subscribeTo(wheels.rwSpeedOutMsg)
    sc_sim.AddModelToTask(task, feedback, 110)

    motor = rwMotorTorque.rwMotorTorque()
    motor.ModelTag = "rwMotorTorque"
    motor.controlAxes_B = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    motor.vehControlInMsg.subscribeTo(feedback.cmdTorqueOutMsg)
    motor.rwParamsInMsg.subscribeTo(rw_config)
    sc_sim.AddModelToTask(task, motor, 105)

    stiction = Rw3Stiction()
    stiction.torqueInMsg.subscribeTo(motor.rwMotorTorqueOutMsg)
    stiction.speedInMsg.subscribeTo(wheels.rwSpeedOutMsg)
    wheels.rwMotorCmdInMsg.subscribeTo(stiction.torqueOutMsg)
    sc_sim.AddModelToTask(task, stiction, 90)

    # Message owners have to outlive the run. The C++ subscribers hold pointers.
    sc_sim.kept_alive = {
        "epoch": epoch_msg,
        "space_weather": space_weather,
        "vehicle": vehicle_msg,
        "thruster_cmd": thruster_cmd,
        "grav_factory": grav_factory,
        "earth": earth,
        "sun": sun,
        "spice": spice,
        "rw_factory": rw_factory,
        "rw_config": rw_config,
    }
    return {
        "sim": sc_sim,
        "sc": sc_object,
        "atmo": atmo,
        "imu": imu,
        "tracker": tracker,
        "css": css_sensors,
        "panels": panels,
        "battery": battery,
        "wheels": wheels,
        "stiction": stiction,
    }


def sample(models, sim_time_ns, fault_flags, sequence):
    state = models["sc"].scStateOutMsg.read()
    sigma = as_floats(state.sigma_BN, 3)
    attitude = as_floats(RigidBodyKinematics.MRP2EP(np.array(sigma)), 4)
    omega = as_floats(state.omega_BN_B, 3)
    wheel_msg = models["wheels"].rwSpeedOutMsg.read()
    wheel_speeds = as_floats(wheel_msg.wheelSpeeds, 3)
    gyro = as_floats(models["imu"].sensorOutMsg.read().AngVelPlatform, 3)
    tracker_q = as_floats(models["tracker"].sensorOutMsg.read().qInrtl2Case, 4)
    css = [float(sensor.cssDataOutMsg.read().OutputData) for sensor in models["css"]]
    power = models["battery"].batPowerOutMsg.read()
    capacity = float(power.storageCapacity)
    level = float(power.storageLevel)
    soc = 0.0 if capacity <= 0.0 else level / capacity
    density = float(models["atmo"].envOutMsgs[0].read().neutralDensity)
    efficiency = float(models["panels"][0].panelEfficiency)
    return pack_state(
        sim_time_ns=sim_time_ns,
        q_bn=attitude,
        omega_bn_b=omega,
        bus_voltage_derived=derived_bus_voltage(soc),
        current_net_power_w=float(power.currentNetPower),
        soc=soc,
        wheel_speeds_rad_s=wheel_speeds,
        gyro_ang_vel_rad_s=gyro,
        star_tracker_q=tracker_q,
        css=css,
        panel_efficiency=efficiency,
        neutral_density_kg_m3=density,
        fault_flags=fault_flags,
        sequence=sequence,
    )


class ZmqPublisher:
    def __init__(self, endpoint):
        try:
            import zmq
        except ImportError as exc:
            raise SystemExit("pyzmq is required unless --no-zmq is set. pip install pyzmq") from exc
        self._zmq = zmq
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 100)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(endpoint)
        # PUB drops messages until a subscriber finishes its handshake.
        time.sleep(0.2)

    def send(self, packet):
        try:
            self._socket.send_multipart([TOPIC, packet], flags=self._zmq.NOBLOCK)
        except self._zmq.Again:
            return

    def close(self):
        self._socket.close(0)
        self._context.term()


def apply_faults(models, sim_time_ns, enabled, brownout_on, gyro_bias):
    flags = 0
    if not enabled:
        models["stiction"].stiction_on = False
        return 0, brownout_on, gyro_bias
    if sim_time_ns >= STICTION_NS:
        models["stiction"].stiction_on = True
        flags |= FAULT_RW3_STICTION
    if sim_time_ns >= BROWNOUT_NS and not brownout_on:
        for panel in models["panels"]:
            panel.panelEfficiency = PANEL_EFFICIENCY * 0.4
        brownout_on = True
    if brownout_on:
        flags |= FAULT_BROWNOUT
    if sim_time_ns >= IMU_WALK_NS:
        step = IMU_WALK_SIGMA * math.sqrt(STEP_S) * np.random.randn(3)
        gyro_bias = np.clip(gyro_bias + step, -IMU_BIAS_LIMIT, IMU_BIAS_LIMIT)
        models["imu"].senRotBias = [float(item) for item in gyro_bias]
        flags |= FAULT_IMU_WALK
    return flags, brownout_on, gyro_bias


def run(duration_s, endpoint, faults, publish, seed):
    if duration_s <= 0.0:
        raise SystemExit("--duration must be positive")
    np.random.seed(seed)
    models = build_simulation()
    sc_sim = models["sim"]
    publisher = ZmqPublisher(endpoint) if publish else None
    sc_sim.InitializeSimulation()
    steps = int(round(duration_s / STEP_S))
    brownout_on = False
    gyro_bias = np.zeros(3)
    stop_ns = 0
    try:
        for sequence in range(steps):
            stop_ns += STEP_NS
            flags, brownout_on, gyro_bias = apply_faults(
                models, stop_ns, faults, brownout_on, gyro_bias
            )
            sc_sim.ConfigureStopTime(stop_ns)
            sc_sim.ExecuteSimulation()
            packet = sample(models, stop_ns, flags, sequence)
            if publisher is not None:
                publisher.send(packet)
            if sequence % 120 == 0 or sequence == steps - 1:
                state = models["sc"].scStateOutMsg.read()
                sigma = as_floats(state.sigma_BN, 3)
                power = models["battery"].batPowerOutMsg.read()
                capacity = float(power.storageCapacity)
                soc = 0.0 if capacity <= 0.0 else float(power.storageLevel) / capacity
                print(
                    f"t={stop_ns * 1.0e-9:8.1f}s"
                    f"  sigma={np.array2string(np.array(sigma), precision=4)}"
                    f"  soc={soc:.3f}"
                    f"  Vderived={derived_bus_voltage(soc):.3f}"
                    f"  flags=0x{flags:04x}"
                )
    finally:
        if publisher is not None:
            publisher.close()
    return steps


def main(argv=None):
    parser = argparse.ArgumentParser(description="3U LEO Basilisk scenario with a nominal inertial pointer.")
    parser.add_argument("--duration", type=float, default=600.0, help="simulation length in seconds")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5555", help="ZeroMQ PUB bind endpoint")
    parser.add_argument("--no-faults", action="store_true", help="leave RW3, the panels, and the IMU at nominal")
    parser.add_argument("--no-zmq", action="store_true", help="run the scenario without opening a ZeroMQ socket")
    parser.add_argument("--seed", type=int, default=7, help="seed for the IMU random walk")
    args = parser.parse_args(argv)
    steps = run(
        duration_s=args.duration,
        endpoint=args.endpoint,
        faults=not args.no_faults,
        publish=not args.no_zmq,
        seed=args.seed,
    )
    where = "without ZeroMQ" if args.no_zmq else f"publisher bound to {args.endpoint}"
    print(f"completed {steps} steps at {STEP_S:.1f} s, {where}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
