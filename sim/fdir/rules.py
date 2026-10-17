"""Executable flight rules for OrbiEdge FDIR.

BusMonitor is pure persistence. FdirManager is the only mode latch.
Both are deterministic and use only the standard library.

Interpretations the mirror must keep:
- Population standard deviation divides by N, not N-1.
- SOC is sampled on every 10th ingest (1-based), which is 1 Hz at 10 Hz.
- Warning priority is overspeed, then jitter, then SOC. Critical beats
  every warning. Critical tie-break is dump-rise, then SOC, then temperature.
- A temperature warning is severity NONE. It sets temp_warn and does not
  change the mode. Severity NONE counts as quiet for the dwell.
- A multi-mode warning opens one 2 s window anchored on the first sample
  of that episode. Later samples of the same warning do not slide it.
  A higher-priority warning replaces a lower-priority window.
- AGREE requires an eligible action and |proposal sim time - record sim time|
  <= 5 s. A larger separation is LATE, even when the proposal is early.
  An ineligible action is REJECT. DISAGREE is reserved and not produced.
- ISOLATE_RW and SAFE stay latched until clear(). A proposal cannot clear
  them. A critical record may escalate ISOLATE_RW to SAFE.
- MOMENTUM_DUMP entry speed is the max abs wheel speed of the sample that
  applied that mode. Dump-rise is a strict increase after 30 s.
- sensors_valid False holds the current mode, disposition HOLD_INVALID,
  and does not apply the record or the proposal.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Sequence

STEP_NS = 100_000_000  # 10 Hz
WHEEL_COUNT = 4
WHEEL_MAX_RAD_S = 628.3185307179587  # Honeywell HR16, 6000 RPM
OVERSPEED_FRACTION = 0.9
OVERSPEED_SAMPLES = 10
DUMP_RISE_NS = 30_000_000_000
STDDEV_HIGH = 1.0
STDDEV_QUIET = 0.2
STDDEV_SAMPLES = 20
SOC_WARN = 0.30
SOC_CRIT = 0.15
SOC_SAMPLES = 5  # at 1 Hz, every 10th 10 Hz sample
TEMP_WARN_C = 40.0
TEMP_CRIT_C = 60.0
TEMP_CRIT_SAMPLES = 50
TEMP_WARN_SAMPLES = 100
QUIET_DWELL_NS = 60_000_000_000
PROPOSAL_WINDOW_NS = 2_000_000_000
PROPOSAL_STALE_NS = 5_000_000_000
SENSOR_LOSS_WALL_S = 0.5

NOMINAL = "NOMINAL"
ISOLATE_RW = "ISOLATE_RW"
MOMENTUM_DUMP = "MOMENTUM_DUMP"
LOAD_SHED = "LOAD_SHED"
SAFE = "SAFE"
MODES = (NOMINAL, ISOLATE_RW, MOMENTUM_DUMP, LOAD_SHED, SAFE)

AGREE = "AGREE"
DISAGREE = "DISAGREE"
LATE = "LATE"
REJECT = "REJECT"
NONE = "NONE"
HOLD_INVALID = "HOLD_INVALID"
DISPOSITIONS = (AGREE, DISAGREE, LATE, REJECT, NONE, HOLD_INVALID)

SEVERITY_NONE = "NONE"
SEVERITY_WARNING = "WARNING"
SEVERITY_CRITICAL = "CRITICAL"

EVT_OVERSPEED = "OVERSPEED"
EVT_DUMP_RISE = "DUMP_RISE"
EVT_JITTER = "JITTER"
EVT_SOC_CRIT = "SOC_CRIT"
EVT_SOC_WARN = "SOC_WARN"
EVT_TEMP_CRIT = "TEMP_CRIT"
EVT_TEMP_WARN = "TEMP_WARN"

# Lower rank is preferred inside one severity. Critical is chosen first.
_WARNING_RANK = {
    EVT_OVERSPEED: 0,
    EVT_JITTER: 1,
    EVT_SOC_WARN: 2,
}
_CRITICAL_RANK = {
    EVT_DUMP_RISE: 0,
    EVT_SOC_CRIT: 1,
    EVT_TEMP_CRIT: 2,
}

_STICKY = frozenset({ISOLATE_RW, SAFE})
_OVERSPEED_LIMIT = OVERSPEED_FRACTION * WHEEL_MAX_RAD_S
_RATE_LIMIT_S = PROPOSAL_WINDOW_NS / 1_000_000_000


def sensor_link_ok(now_wall_s: float, last_sensor_wall_s: float | None) -> bool:
    """False after the sensor link has been quiet longer than 0.5 s."""
    if last_sensor_wall_s is None:
        return False
    return (float(now_wall_s) - float(last_sensor_wall_s)) <= SENSOR_LOSS_WALL_S


def sensor_sample(
    sim_time_ns: int,
    *,
    soc: float = 0.8,
    solar_scale: float = 1.0,
    wheel_speed_rad_s: Sequence[float] | None = None,
    temp_c: float = 20.0,
    heater_on: bool = False,
) -> dict:
    """One 10 Hz sensor object. Wheel order is 0..3."""
    if wheel_speed_rad_s is None:
        wheels = [0.0, 0.0, 0.0, 0.0]
    else:
        if len(wheel_speed_rad_s) != WHEEL_COUNT:
            raise ValueError(f"wheel_speed_rad_s needs {WHEEL_COUNT} values")
        wheels = [float(speed) for speed in wheel_speed_rad_s]
    return {
        "sim_time_ns": int(sim_time_ns),
        "soc": float(soc),
        "solar_scale": float(solar_scale),
        "wheel_speed_rad_s": wheels,
        "temp_c": float(temp_c),
        "heater_on": bool(heater_on),
    }


@dataclass
class FaultRecord:
    sim_time_ns: int
    eligible: tuple[str, ...]
    default_mode: str | None
    severity: str
    wheel_index: int
    failed: list[tuple[str, float, float]]
    event: str = ""
    temp_warn: bool = False


@dataclass(frozen=True)
class Decision:
    mode: str
    wheel_index: int
    disposition: str
    event: str


@dataclass
class _Window:
    anchor_ns: int
    eligible: tuple[str, ...]
    default_mode: str
    wheel_index: int
    event: str


def _population_stddev(values: deque[float]) -> float:
    """Population stddev. The divisor is N, not N-1."""
    count = len(values)
    if count == 0:
        return 0.0
    mean = sum(values) / count
    variance = sum((value - mean) ** 2 for value in values) / count
    if variance <= 0.0:
        return 0.0
    return variance ** 0.5


def _fastest(speeds: Sequence[float], indexes: Sequence[int] | None = None) -> int:
    pool = range(WHEEL_COUNT) if indexes is None else indexes
    best_index = 0 if indexes is None else indexes[0]
    best_mag = -1.0
    for index in pool:
        mag = abs(speeds[index])
        if mag > best_mag:
            best_mag = mag
            best_index = index
    return best_index


class BusMonitor:
    """Persistence counters. ingest() does not change the applied mode."""

    def __init__(self) -> None:
        self._over_count = [0] * WHEEL_COUNT
        self._speed_hist = [deque(maxlen=STDDEV_SAMPLES) for _ in range(WHEEL_COUNT)]
        self._seen = 0
        self._soc_crit = 0
        self._soc_warn = 0
        self._temp_crit = 0
        self._temp_warn = 0
        self._prev_ns: int | None = None
        self._applied_mode = NOMINAL
        self._applied_wheel = -1
        self._dump_entry_ns: int | None = None
        self._dump_entry_speed = 0.0
        self._last_max_abs = 0.0

    @property
    def last_sim_time_ns(self) -> int:
        if self._prev_ns is None:
            return 0
        return self._prev_ns

    def note_applied(self, mode: str, sim_time_ns: int, wheel_index: int) -> None:
        """Record the mode FdirManager actually applied.

        Dump-rise latches the entry snapshot only on the transition into
        MOMENTUM_DUMP. Leaving the mode drops that snapshot.
        """
        if mode == MOMENTUM_DUMP and self._applied_mode != MOMENTUM_DUMP:
            self._dump_entry_ns = int(sim_time_ns)
            self._dump_entry_speed = self._last_max_abs
        elif mode != MOMENTUM_DUMP:
            self._dump_entry_ns = None
            self._dump_entry_speed = 0.0
        self._applied_mode = mode
        self._applied_wheel = int(wheel_index)

    def ingest(self, sample: dict) -> FaultRecord:
        sim_time_ns, soc, _solar, speeds, temp_c = self._read(sample)
        if self._prev_ns is not None and sim_time_ns - self._prev_ns != STEP_NS:
            # Consecutive-sample predicates do not bridge a missed 10 Hz step.
            self._over_count = [0] * WHEEL_COUNT
            self._temp_crit = 0
            self._temp_warn = 0
            for hist in self._speed_hist:
                hist.clear()
        self._prev_ns = sim_time_ns
        self._last_max_abs = max(abs(speed) for speed in speeds)

        for index, speed in enumerate(speeds):
            if abs(speed) > _OVERSPEED_LIMIT:
                self._over_count[index] += 1
            else:
                self._over_count[index] = 0
            self._speed_hist[index].append(speed)

        self._seen += 1
        if self._seen % 10 == 0:
            self._observe_soc(soc)

        if temp_c >= TEMP_CRIT_C:
            self._temp_crit += 1
            self._temp_warn = 0
        elif temp_c >= TEMP_WARN_C:
            self._temp_warn += 1
            self._temp_crit = 0
        else:
            self._temp_crit = 0
            self._temp_warn = 0

        candidates: list[dict] = []
        persistent = [
            index for index in range(WHEEL_COUNT) if self._over_count[index] >= OVERSPEED_SAMPLES
        ]
        if persistent:
            wheel = _fastest(speeds, persistent)
            failed = [
                (f"wheel_speed_rad_s[{index}]", abs(speeds[index]), _OVERSPEED_LIMIT)
                for index in persistent
            ]
            candidates.append(
                self._candidate(
                    SEVERITY_WARNING,
                    (MOMENTUM_DUMP, SAFE),
                    MOMENTUM_DUMP,
                    wheel,
                    EVT_OVERSPEED,
                    failed,
                    _WARNING_RANK[EVT_OVERSPEED],
                )
            )

        if (
            self._applied_mode == MOMENTUM_DUMP
            and self._dump_entry_ns is not None
            and sim_time_ns - self._dump_entry_ns >= DUMP_RISE_NS
            and self._last_max_abs > self._dump_entry_speed
        ):
            candidates.append(
                self._candidate(
                    SEVERITY_CRITICAL,
                    (SAFE,),
                    SAFE,
                    _fastest(speeds),
                    EVT_DUMP_RISE,
                    [("wheel_speed_max_abs", self._last_max_abs, self._dump_entry_speed)],
                    _CRITICAL_RANK[EVT_DUMP_RISE],
                )
            )

        if all(len(hist) == STDDEV_SAMPLES for hist in self._speed_hist):
            deviations = [_population_stddev(hist) for hist in self._speed_hist]
            loud = [index for index, dev in enumerate(deviations) if dev > STDDEV_HIGH]
            quiet = [index for index, dev in enumerate(deviations) if dev < STDDEV_QUIET]
            if len(loud) == 1 and len(quiet) == 3:
                wheel = loud[0]
                candidates.append(
                    self._candidate(
                        SEVERITY_WARNING,
                        (ISOLATE_RW, NOMINAL),
                        ISOLATE_RW,
                        wheel,
                        EVT_JITTER,
                        [(f"wheel_speed_stddev[{wheel}]", deviations[wheel], STDDEV_HIGH)],
                        _WARNING_RANK[EVT_JITTER],
                    )
                )

        if self._soc_crit >= SOC_SAMPLES:
            candidates.append(
                self._candidate(
                    SEVERITY_CRITICAL,
                    (SAFE,),
                    SAFE,
                    -1,
                    EVT_SOC_CRIT,
                    [("soc", soc, SOC_CRIT)],
                    _CRITICAL_RANK[EVT_SOC_CRIT],
                )
            )
        elif self._soc_warn >= SOC_SAMPLES:
            candidates.append(
                self._candidate(
                    SEVERITY_WARNING,
                    (LOAD_SHED, NOMINAL),
                    LOAD_SHED,
                    -1,
                    EVT_SOC_WARN,
                    [("soc", soc, SOC_WARN)],
                    _WARNING_RANK[EVT_SOC_WARN],
                )
            )

        if self._temp_crit >= TEMP_CRIT_SAMPLES:
            candidates.append(
                self._candidate(
                    SEVERITY_CRITICAL,
                    (SAFE,),
                    SAFE,
                    -1,
                    EVT_TEMP_CRIT,
                    [("temp_c", temp_c, TEMP_CRIT_C)],
                    _CRITICAL_RANK[EVT_TEMP_CRIT],
                )
            )

        temp_warn = self._temp_warn >= TEMP_WARN_SAMPLES
        temp_failed = [("temp_c", temp_c, TEMP_WARN_C)] if temp_warn else []
        if not candidates:
            return FaultRecord(
                sim_time_ns=sim_time_ns,
                eligible=(),
                default_mode=None,
                severity=SEVERITY_NONE,
                wheel_index=-1,
                failed=list(temp_failed),
                event=EVT_TEMP_WARN if temp_warn else "",
                temp_warn=temp_warn,
            )

        winner = candidates[0]
        for candidate in candidates[1:]:
            if _preferred(candidate, winner):
                winner = candidate
        failed: list[tuple[str, float, float]] = list(winner["failed"])
        for candidate in candidates:
            if candidate is winner:
                continue
            failed.extend(candidate["failed"])
        failed.extend(temp_failed)
        return FaultRecord(
            sim_time_ns=sim_time_ns,
            eligible=winner["eligible"],
            default_mode=winner["default_mode"],
            severity=winner["severity"],
            wheel_index=winner["wheel_index"],
            failed=failed,
            event=winner["event"],
            temp_warn=temp_warn,
        )

    def _observe_soc(self, soc: float) -> None:
        if soc < SOC_CRIT:
            self._soc_crit += 1
            self._soc_warn += 1
        elif soc < SOC_WARN:
            self._soc_crit = 0
            self._soc_warn += 1
        else:
            self._soc_crit = 0
            self._soc_warn = 0

    @staticmethod
    def _candidate(
        severity: str,
        eligible: tuple[str, ...],
        default_mode: str,
        wheel_index: int,
        event: str,
        failed: list[tuple[str, float, float]],
        order: int,
    ) -> dict:
        return {
            "severity": severity,
            "eligible": eligible,
            "default_mode": default_mode,
            "wheel_index": wheel_index,
            "event": event,
            "failed": failed,
            "order": order,
        }

    @staticmethod
    def _read(sample: dict) -> tuple[int, float, float, list[float], float]:
        try:
            sim_time_ns = sample["sim_time_ns"]
            soc = sample["soc"]
            solar_scale = sample["solar_scale"]
            wheels = sample["wheel_speed_rad_s"]
            temp_c = sample["temp_c"]
            sample["heater_on"]
        except KeyError as exc:
            raise ValueError(f"sensor missing {exc.args[0]}") from exc
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
            raise ValueError("sim_time_ns must be an int")
        if len(wheels) != WHEEL_COUNT:
            raise ValueError(f"wheel_speed_rad_s needs {WHEEL_COUNT} values")
        return (
            sim_time_ns,
            float(soc),
            float(solar_scale),
            [float(speed) for speed in wheels],
            float(temp_c),
        )


def _preferred(candidate: dict, incumbent: dict) -> bool:
    """Critical outranks warning. Inside one severity, lower order wins."""
    rank = {SEVERITY_WARNING: 1, SEVERITY_CRITICAL: 2}
    if rank[candidate["severity"]] != rank[incumbent["severity"]]:
        return rank[candidate["severity"]] > rank[incumbent["severity"]]
    return candidate["order"] < incumbent["order"]


def _coerce_proposal(proposal: dict | None) -> dict | None:
    if proposal is None:
        return None
    if not isinstance(proposal, dict):
        return {"invalid": True, "sim_time_ns": None, "action": "", "wheel_index": -1}
    try:
        action = proposal["action"]
        wheel = proposal["wheel_index"]
        confidence = proposal["confidence"]
        sim_time_ns = proposal["sim_time_ns"]
    except (KeyError, TypeError):
        return {"invalid": True, "sim_time_ns": None, "action": "", "wheel_index": -1}
    invalid = False
    if action not in MODES:
        invalid = True
        action = action if isinstance(action, str) else ""
    if isinstance(wheel, bool) or not isinstance(wheel, int) or not 0 <= wheel < WHEEL_COUNT:
        invalid = True
        wheel = -1
    if (
        isinstance(confidence, bool)
        or not isinstance(confidence, (int, float))
        or not 0.0 <= float(confidence) <= 1.0
    ):
        invalid = True
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
        invalid = True
        sim_time_ns = None
    return {
        "invalid": invalid,
        "action": action,
        "wheel_index": wheel,
        "sim_time_ns": sim_time_ns,
    }


def _proposal_late(record_ns: int, proposal: dict) -> bool:
    sim_time_ns = proposal.get("sim_time_ns")
    if not isinstance(sim_time_ns, int):
        return False
    return abs(record_ns - sim_time_ns) > PROPOSAL_STALE_NS


class FdirManager:
    """Apply a FaultRecord. Proposals never clear a sticky mode."""

    def __init__(self, monitor: BusMonitor | None = None) -> None:
        self.monitor = monitor if monitor is not None else BusMonitor()
        self.mode = NOMINAL
        self.wheel_index = -1
        self._window: _Window | None = None
        self._responded: str | None = None
        self._quiet_since: int | None = None

    def clear(self) -> Decision:
        """Human FDIR.CLEAR. This is the only exit from ISOLATE_RW and SAFE."""
        self._window = None
        self._responded = None
        self._quiet_since = None
        self.mode = NOMINAL
        self.wheel_index = -1
        self.monitor.note_applied(NOMINAL, self.monitor.last_sim_time_ns, -1)
        return Decision(NOMINAL, -1, NONE, "CLEAR")

    def tick(self, record: FaultRecord, proposal: dict | None, sensors_valid: bool) -> Decision:
        if not sensors_valid:
            return Decision(self.mode, self.wheel_index, HOLD_INVALID, record.event)

        proposal_n = _coerce_proposal(proposal)
        if record.severity == SEVERITY_CRITICAL:
            self._window = None
            self._quiet_since = None
            if record.default_mode is not None and self._can_apply(record.default_mode):
                self._apply(record.default_mode, record.wheel_index, record.sim_time_ns)
            self._responded = record.event
            return Decision(self.mode, self.wheel_index, NONE, record.event)

        if self._window is not None and _warning_outranks(record, self._window):
            # A step into overspeed can look like jitter for one window.
            # Overspeed is preferred and must still be able to open its own.
            self._window = None
            self._responded = None

        if self._window is not None:
            return self._tick_window(record, proposal_n)

        if record.severity == SEVERITY_WARNING:
            return self._tick_warning(record, proposal_n)

        self._responded = None
        return self._tick_quiet(record)

    def _tick_warning(self, record: FaultRecord, proposal: dict | None) -> Decision:
        self._quiet_since = None
        if self._responded not in (None, record.event):
            self._responded = None
        if self._responded == record.event:
            return Decision(self.mode, self.wheel_index, NONE, record.event)
        if record.eligible and not any(self._can_apply(mode) for mode in record.eligible):
            return Decision(self.mode, self.wheel_index, NONE, record.event)
        if len(record.eligible) == 1:
            mode = record.eligible[0]
            if self._can_apply(mode):
                self._apply(mode, record.wheel_index, record.sim_time_ns)
            self._responded = record.event
            return Decision(self.mode, self.wheel_index, NONE, record.event)
        if len(record.eligible) > 1 and record.default_mode is not None:
            self._window = _Window(
                anchor_ns=record.sim_time_ns,
                eligible=record.eligible,
                default_mode=record.default_mode,
                wheel_index=record.wheel_index,
                event=record.event,
            )
            return self._tick_window(record, proposal)
        return Decision(self.mode, self.wheel_index, NONE, record.event)

    def _tick_window(self, record: FaultRecord, proposal: dict | None) -> Decision:
        window = self._window
        assert window is not None
        deadline = window.anchor_ns + PROPOSAL_WINDOW_NS
        expired = record.sim_time_ns >= deadline
        if proposal is not None:
            disposition = self._classify(proposal, record, window)
            if (
                disposition == AGREE
                and record.sim_time_ns <= deadline
                and self._can_apply(proposal["action"])
            ):
                wheel = proposal["wheel_index"] if proposal["action"] == ISOLATE_RW else window.wheel_index
                self._apply(proposal["action"], wheel, record.sim_time_ns)
                self._responded = window.event
                self._window = None
                return Decision(self.mode, self.wheel_index, AGREE, window.event)
            if disposition == AGREE and not self._can_apply(proposal["action"]):
                # Eligible on paper, but a proposal is not allowed to clear.
                disposition = REJECT
            if not expired:
                return Decision(self.mode, self.wheel_index, disposition, window.event)
        if expired:
            if self._can_apply(window.default_mode):
                self._apply(window.default_mode, window.wheel_index, record.sim_time_ns)
            self._responded = window.event
            self._window = None
            return Decision(self.mode, self.wheel_index, NONE, window.event)
        return Decision(self.mode, self.wheel_index, NONE, window.event)

    def _classify(self, proposal: dict, record: FaultRecord, window: _Window) -> str:
        if _proposal_late(record.sim_time_ns, proposal):
            return LATE
        if proposal.get("invalid"):
            return REJECT
        if proposal["action"] in window.eligible:
            return AGREE
        return REJECT

    def _tick_quiet(self, record: FaultRecord) -> Decision:
        if self.mode == NOMINAL or self.mode in _STICKY:
            self._quiet_since = None
            return Decision(self.mode, self.wheel_index, NONE, record.event)
        if self._quiet_since is None:
            self._quiet_since = record.sim_time_ns
        elif record.sim_time_ns - self._quiet_since >= QUIET_DWELL_NS:
            self._apply(NOMINAL, -1, record.sim_time_ns)
            self._quiet_since = None
        return Decision(self.mode, self.wheel_index, NONE, record.event)

    def _can_apply(self, new_mode: str) -> bool:
        if new_mode not in MODES:
            return False
        if new_mode == self.mode:
            return True
        if self.mode == ISOLATE_RW and new_mode == SAFE:
            return True
        if self.mode in _STICKY:
            return False
        return True

    def _apply(self, new_mode: str, wheel_index: int, sim_time_ns: int) -> None:
        if new_mode in (ISOLATE_RW, MOMENTUM_DUMP):
            self.wheel_index = wheel_index
        else:
            self.wheel_index = -1
        self.mode = new_mode
        self.monitor.note_applied(self.mode, sim_time_ns, self.wheel_index)


def _warning_outranks(record: FaultRecord, window: _Window) -> bool:
    if record.severity != SEVERITY_WARNING:
        return False
    new_rank = _WARNING_RANK.get(record.event)
    old_rank = _WARNING_RANK.get(window.event)
    if new_rank is None or old_rank is None:
        return False
    return new_rank < old_rank


def uplink_filter(
    opcode: str,
    proposal: dict | None,
    now_sim_ns: int,
    now_wall_s: float,
    last_accept_wall_s: float | None,
) -> tuple[bool, str]:
    """Gate AGENT.PROPOSE. This function does not change a mode.

    FDIR.CLEAR is dropped here; the manager accepts clear() on the vehicle.
    A proposal older than 5 s of sim time is stale. A second acceptance
    inside 2.0 s of wall time is dropped. None for last_accept_wall_s
    means no earlier acceptance.
    """
    op = str(opcode)
    if op == "FDIR.CLEAR":
        return False, "dropped_clear"
    if op != "AGENT.PROPOSE":
        return False, "dropped_opcode"
    coerced = _coerce_proposal(proposal)
    if coerced is None or coerced.get("invalid"):
        return False, "invalid_proposal"
    proposal_ns = coerced["sim_time_ns"]
    if not isinstance(proposal_ns, int) or now_sim_ns - proposal_ns > PROPOSAL_STALE_NS:
        return False, "stale"
    if last_accept_wall_s is not None and (float(now_wall_s) - float(last_accept_wall_s)) < _RATE_LIMIT_S:
        return False, "rate_limit"
    return True, "ok"
