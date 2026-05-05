# src/orca_v1/apply_sequence.py
"""
apply_sequence.py

Two apply architectures live here side-by-side:

V1 — kinematic-then-haptic (single spring soft bumper, baseline+delta detection):
    ApplyCycleResult, program_motions(), autozero_and_home(), run_apply_cycle()

V2 — technical-note haptic-only (three springs always on, three-condition detection):
    ApplyV2CycleResult, run_v2_apply_cycle()

autozero_and_home() is shared: both V1 and V2 call it after boot to establish
home_um.  In V2, configure_haptic_effects() is called immediately after
autozero_and_home() returns (done in main.py, not here).

V2 control loop invariants:
  - Motor stays in HapticMode for the entire session after configure_haptic_effects().
  - The only runtime register write is set_constant_force() via the state machine.
  - keep_haptic_effects_alive() is called every tick regardless of state.
  - client.reset_velocity_tracker() is called at the start of each cycle.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import time

from pyorcasdk import MotorMode

from .config import AppConfig
from .orca_client import OrcaClient, OrcaDeviceError
from .logging_csv import CsvLogger, CsvLogConfig
from .haptics import (
    enter_soft_bumper,
    keep_soft_bumper_alive,
    build_soft_bumper_config_from_app_config,
    keep_haptic_effects_alive,
)
from .contact_detect import (
    build_contact_detector_from_app_config,
    build_v2_contact_detector,
    build_v2_empty_detector,
)
from .state_machine import ControlState, LabelApplicatorStateMachine


# ============================================================
# Result types
# ============================================================

@dataclass(frozen=True)
class ApplyCycleResult:
    """Result from a V1 apply cycle."""
    ok: bool
    reason: str
    home_um: int
    apply_target_um: int
    haptic_entry_um: int
    baseline_mN: Optional[int]
    peak_force_mN: int
    contact_delta_mN: Optional[int]
    elapsed_s: float
    csv_path: Optional[str]


@dataclass(frozen=True)
class ApplyV2CycleResult:
    """
    Result from a V2 apply cycle (run_v2_apply_cycle).

    reason values:
        "CONTACT"        — contact detected and hold completed; shaft returned home.
        "EMPTY"          — no label found; shaft reached Spring B and returned home.
        "APPLY_TIMEOUT"  — apply stroke took longer than return_timeout_s; aborted.
        "RETURN_TIMEOUT" — shaft did not reach home within return_timeout_s after RETURN_HOME.
        "INTERRUPTED"    — KeyboardInterrupt caught.
        "ORCA_ERROR"     — SDK error during cycle.
        "ERROR"          — unexpected exception.
    """
    ok: bool
    reason: str
    contact_detected: bool
    apply_force_mN: int
    home_um: int
    spring_b_position_um: int
    elapsed_s: float
    csv_path: Optional[str]


# ============================================================
# Shared helpers
# ============================================================

def _wait_reach_position(
    client: OrcaClient,
    target_um: int,
    tol_um: int,
    timeout_s: float,
    poll_s: float = 0.02,
) -> Optional[int]:
    """
    Blocking-ish wait until shaft reaches target_um within tol_um.
    Uses stream cache when available; falls back to blocking read.
    Returns the position reached, or None on timeout.
    Not used in the V2 high-speed loop (V2 checks position inline).
    """
    t0 = time.time()
    while (time.time() - t0) < timeout_s:
        try:
            client.run()
            snap = client.read_stream_cache()
            pos = snap.position_um
        except Exception:
            pos = None

        if pos is None:
            try:
                pos = client.get_position_um_blocking()
            except Exception:
                pos = None

        if pos is not None and abs(int(pos) - int(target_um)) <= int(tol_um):
            return int(pos)

        time.sleep(poll_s)

    return None


# ============================================================
# Shared: AutoZero + Home (used by both V1 and V2)
# ============================================================

def program_motions(client: OrcaClient, cfg: AppConfig, home_um: int) -> None:
    """
    Program kinematic motions for V1 (HOME, RETURN, APPLY).
    Not called by V2 — V2 uses haptic forces exclusively after AutoZero.
    """
    m = cfg.motion
    apply_target_um = home_um + m.apply_travel_um

    client.set_kinematic_motion(
        motion_id=m.motion_home_id,
        target_um=home_um,
        time_ms=m.motion_home_time_ms,
        delay_ms=0,
        motion_type=m.kin_type_min_jerk,
        auto_next=0,
        next_id=0,
    )

    client.set_kinematic_motion(
        motion_id=m.motion_return_id,
        target_um=home_um,
        time_ms=m.motion_return_time_ms,
        delay_ms=0,
        motion_type=m.kin_type_min_jerk,
        auto_next=0,
        next_id=0,
    )

    client.set_kinematic_motion(
        motion_id=m.motion_apply_id,
        target_um=apply_target_um,
        time_ms=m.apply_time_ms,
        delay_ms=0,
        motion_type=m.kin_type_min_jerk,
        auto_next=0,
        next_id=0,
    )


def autozero_and_home(client: OrcaClient, cfg: AppConfig) -> int:
    """
    AutoZero the motor, compute home_um, and move the shaft to home.

    Shared by V1 and V2.  In V1, also programs kinematic motions.
    In V2, caller must call configure_haptic_effects(client, la_cfg, home_um)
    immediately after this returns — before entering the IDLE loop.

    Returns:
        home_um: absolute home position in micrometres.
    """
    client.set_mode(MotorMode.AutoZeroMode)
    client.wait_until_not_mode(
        MotorMode.AutoZeroMode,
        timeout_s=cfg.timeouts.autozero_timeout_s,
    )

    pos_after_um = client.get_position_um_blocking()
    home_um = pos_after_um + cfg.motion.home_offset_um

    # V1: program kinematic motions now
    # V2: haptic effects are configured by the caller after this returns
    program_motions(client, cfg, home_um=home_um)

    client.set_mode(MotorMode.KinematicMode)
    time.sleep(0.1)

    client.trigger_kinematic_motion(cfg.motion.motion_home_id)

    reached = _wait_reach_position(
        client,
        target_um=home_um,
        tol_um=cfg.tolerances.home_tol_um,
        timeout_s=cfg.timeouts.home_timeout_s,
    )
    if reached is None:
        print("[AUTOZERO_HOME] Warning: HOME not reached within tolerance/time (continuing).")
    else:
        print(f"[AUTOZERO_HOME] HOME reached. pos={reached} um  home_um={home_um} um")

    return home_um


# ============================================================
# V1 — Kinematic-then-haptic apply cycle
# ============================================================

def run_apply_cycle(client: OrcaClient, cfg: AppConfig, home_um: int) -> ApplyCycleResult:
    """
    V1 apply cycle: kinematic APPLY -> haptic soft bumper -> contact detection
    -> kinematic RETURN -> home reached.

    Preserved unchanged from V1. Not used by V2.
    """
    m = cfg.motion
    control_hz = cfg.loop.control_hz
    dt = 1.0 / float(control_hz)

    apply_target_um = home_um + m.apply_travel_um
    haptic_entry_um = home_um + (m.apply_travel_um - m.haptic_before_limit_um)

    client.set_mode(MotorMode.KinematicMode)
    client.trigger_kinematic_motion(m.motion_apply_id)

    alive = client.wait_for_stream_alive(timeout_s=2.0)
    if not alive:
        print("[APPLY] Warning: stream_cache did not show updates during warmup window.")

    csv_cfg = CsvLogConfig(
        log_dir=cfg.logging.log_dir,
        prefix=cfg.logging.csv_prefix,
        include_header=True,
    )
    logger = CsvLogger(csv_cfg).open()

    det = build_contact_detector_from_app_config(cfg)
    bumper_cfg = build_soft_bumper_config_from_app_config(cfg)

    t0 = time.time()
    last_t = t0
    last_pos: Optional[int] = None

    phase = "APPLY_KIN"
    haptic_entered = False
    baseline_mN: Optional[int] = None
    peak_force = -(10**9)
    contact_delta: Optional[int] = None

    reason = "UNKNOWN"
    ok = False

    try:
        while True:
            now = time.time()
            elapsed = now - t0

            client.run()
            snap = client.read_stream_cache()
            pos_um = snap.position_um
            force_mN = snap.force_mN
            peak_force = max(peak_force, force_mN)

            if client.is_stream_stale(snap):
                logger.log(
                    epoch_s=now, phase=phase, elapsed_s=elapsed,
                    position_um=pos_um, velocity_um_s=0.0,
                    force_raw_mN=force_mN, baseline_mN=baseline_mN,
                    event="STREAM_NOT_READY",
                )
                time.sleep(dt)
                continue

            vel = 0.0
            if last_pos is not None:
                dtt = max(1e-6, now - last_t)
                vel = (pos_um - last_pos) / dtt
            last_pos = pos_um
            last_t = now

            event = ""

            if (not haptic_entered) and (pos_um >= haptic_entry_um):
                phase = "HAPTIC_BUMPER"
                enter_soft_bumper(client, cfg=bumper_cfg, center_um=pos_um)
                baseline_mN = force_mN
                det.capture_baseline(force_mN, now)
                haptic_entered = True
                event = f"ENTER_HAPTIC baseline={baseline_mN}"

            if phase == "HAPTIC_BUMPER":
                keep_soft_bumper_alive(client)
                res = det.update(force_mN, now)
                if res.triggered:
                    event = "CONTACT_TRIGGER"
                    contact_delta = res.delta_mN
                    client.set_mode(MotorMode.KinematicMode)
                    client.trigger_kinematic_motion(m.motion_return_id)
                    phase = "RETURN_HOME"

            if phase in ("APPLY_KIN", "HAPTIC_BUMPER"):
                if pos_um >= (apply_target_um - 500):
                    event = "APPLY_REACHED_TARGET"
                    client.set_mode(MotorMode.KinematicMode)
                    client.trigger_kinematic_motion(m.motion_return_id)
                    phase = "RETURN_HOME"

            if phase == "RETURN_HOME":
                if abs(pos_um - home_um) <= cfg.tolerances.home_tol_um:
                    event = (event + "|" if event else "") + "HOME_REACHED"
                    ok = True
                    reason = "COMPLETED"
                    logger.log(
                        epoch_s=now, phase=phase, elapsed_s=elapsed,
                        position_um=pos_um, velocity_um_s=vel,
                        force_raw_mN=force_mN, baseline_mN=baseline_mN,
                        event=event,
                    )
                    break

                if elapsed > (cfg.timeouts.return_timeout_s + 30.0):
                    reason = "RETURN_TIMEOUT"
                    ok = False
                    break

            logger.log(
                epoch_s=now, phase=phase, elapsed_s=elapsed,
                position_um=pos_um, velocity_um_s=vel,
                force_raw_mN=force_mN, baseline_mN=baseline_mN,
                event=event,
            )
            time.sleep(dt)

    except KeyboardInterrupt:
        ok = False; reason = "INTERRUPTED"
    except OrcaDeviceError as ex:
        ok = False; reason = f"ORCA_ERROR: {ex}"
    except Exception as ex:
        ok = False; reason = f"ERROR: {ex}"
    finally:
        csv_path = str(logger.path) if logger.path else None
        logger.close()
        if not ok and phase != "RETURN_HOME":
            try:
                client.set_mode(MotorMode.KinematicMode)
                client.trigger_kinematic_motion(m.motion_return_id)
            except Exception:
                pass

    return ApplyCycleResult(
        ok=ok,
        reason=reason,
        home_um=home_um,
        apply_target_um=apply_target_um,
        haptic_entry_um=haptic_entry_um,
        baseline_mN=baseline_mN,
        peak_force_mN=peak_force if peak_force != -(10**9) else 0,
        contact_delta_mN=contact_delta,
        elapsed_s=time.time() - t0,
        csv_path=csv_path,
    )


# ============================================================
# V2 — Haptic-only apply cycle (technical note architecture)
# ============================================================

def run_v2_apply_cycle(
    client: OrcaClient,
    cfg: AppConfig,
    home_um: int,
    sm: LabelApplicatorStateMachine,
) -> ApplyV2CycleResult:
    """
    Execute one V2 apply cycle using the haptic-only architecture.

    The motor must already be in HapticMode with effects configured via
    configure_haptic_effects(). The state machine (sm) must be in
    RETURN_HOME state with constant force = 0 before calling this.

    Cycle flow:
        RETURN_HOME -> APPLY (via sm.enter_state)
        APPLY -> HOLD_CONTACT (contact detected) or RETURN_HOME (empty/timeout)
        HOLD_CONTACT -> RETURN_HOME (hold timer expires)
        RETURN_HOME: wait for shaft to reach home, then return result.

    The state machine is reused across cycles. On normal completion it ends
    in RETURN_HOME ready for the next call. On error, safe_return() is called.

    Args:
        client:  OrcaClient with open port in HapticMode.
        cfg:     Full AppConfig (uses loop.control_hz, tolerances, timeouts,
                 logging, and label_applicator sections).
        home_um: Absolute home position from autozero_and_home().
        sm:      LabelApplicatorStateMachine created once after AutoZero.

    Returns:
        ApplyV2CycleResult describing the outcome of this cycle.

    Raises:
        ValueError: if cfg.label_applicator is None (V2 section not configured).
    """
    if cfg.label_applicator is None:
        raise ValueError(
            "run_v2_apply_cycle requires cfg.label_applicator to be configured. "
            "Add a label_applicator: section to your YAML."
        )

    la = cfg.label_applicator
    dt = 1.0 / float(cfg.loop.control_hz)
    home_tol_um = cfg.tolerances.home_tol_um
    return_timeout_s = cfg.timeouts.return_timeout_s

    # Spring B absolute position — used for empty detection and logging
    spring_b_position_um = home_um + la.extended_position_offset_um

    # Safety guard: state machine must start in RETURN_HOME
    if sm.state != ControlState.RETURN_HOME:
        print(f"[V2 APPLY] Warning: state machine in {sm.state.name} at cycle start "
              f"(expected RETURN_HOME). Calling safe_return() before proceeding.")
        sm.safe_return()

    # Reset velocity tracker so the first sample is not corrupted by
    # position delta from wherever the shaft was at the end of the last cycle.
    client.reset_velocity_tracker()

    # Build per-cycle detectors (stateful hit counters reset each cycle)
    contact_det = build_v2_contact_detector(la, home_um)
    empty_det = build_v2_empty_detector(la, home_um)

    # Open CSV logger
    csv_cfg = CsvLogConfig(
        log_dir=cfg.logging.log_dir,
        prefix=cfg.logging.csv_prefix + "_v2",
        include_header=True,
    )
    logger = CsvLogger(csv_cfg).open()

    # Cycle state
    t0 = time.time()
    contact_detected = False
    return_start_s: Optional[float] = None
    reason = "UNKNOWN"
    ok = False

    def _log(pos_um, force_mN, vel_um_s, event=""):
        """Inline logging helper. Uses apply_force_mN as 'baseline' and
        (force - apply_force) as 'delta' — meaningful V2 diagnostics."""
        now_log = time.time()
        logger.log(
            epoch_s=now_log,
            phase=sm.state.name,
            elapsed_s=now_log - t0,
            position_um=pos_um,
            velocity_um_s=vel_um_s,
            force_raw_mN=force_mN,
            baseline_mN=la.apply_force_mN,
            delta_mN=force_mN - la.apply_force_mN,
            event=event,
        )

    try:
        # ---- Warm up stream ----
        alive = client.wait_for_stream_alive(timeout_s=2.0)
        if not alive:
            print("[V2 APPLY] Warning: stream cache did not update during warmup.")

        # ---- Trigger apply stroke ----
        sm.enter_state(ControlState.APPLY)

        # ---- Main control loop ----
        while True:
            now = time.time()

            # Keep comms alive and refresh stream cache
            client.run()
            snap = client.read_stream_cache()

            pos_um = snap.position_um
            force_mN = snap.force_mN
            vel_um_s = snap.velocity_um_s

            # Skip stale frames — stream not ready yet
            if client.is_stream_stale(snap):
                _log(pos_um, force_mN, vel_um_s, event="STREAM_NOT_READY")
                time.sleep(dt)
                continue

            # Keep haptic effects alive every tick (watchdog requirement)
            keep_haptic_effects_alive(client)

            current_state = sm.state
            event = ""

            # ---- APPLY state: detect contact or empty ----
            if current_state == ControlState.APPLY:
                elapsed = now - t0

                cr = contact_det.update(force_mN, vel_um_s, pos_um)

                if cr.triggered:
                    contact_detected = True
                    event = (
                        f"CONTACT_DETECTED "
                        f"f={force_mN} v={vel_um_s:.0f} p={pos_um} "
                        f"hits={cr.consecutive_hits}"
                    )
                    _log(pos_um, force_mN, vel_um_s, event=event)
                    sm.enter_state(ControlState.HOLD_CONTACT)
                    time.sleep(dt)
                    continue

                else:
                    # Log which conditions are blocking (useful during tuning)
                    cond = (
                        f"f={'Y' if cr.force_ok else 'N'}"
                        f" v={'Y' if cr.velocity_ok else 'N'}"
                        f" p={'Y' if cr.position_ok else 'N'}"
                    )

                    er = empty_det.update(vel_um_s, pos_um)
                    if er.triggered:
                        event = f"EMPTY_DETECTED p={pos_um} v={vel_um_s:.0f}"
                        _log(pos_um, force_mN, vel_um_s, event=event)
                        sm.enter_state(ControlState.RETURN_HOME)
                        return_start_s = now
                        time.sleep(dt)
                        continue

                    # Apply timeout guard: shaft has taken too long — abort safely
                    if elapsed > return_timeout_s:
                        event = f"APPLY_TIMEOUT elapsed={elapsed:.2f}s"
                        _log(pos_um, force_mN, vel_um_s, event=event)
                        sm.enter_state(ControlState.RETURN_HOME)
                        return_start_s = now
                        reason = "APPLY_TIMEOUT"
                        ok = False
                        time.sleep(dt)
                        continue

                    _log(pos_um, force_mN, vel_um_s, event=cond)

            # ---- HOLD_CONTACT: wait for hold timer to expire ----
            elif current_state == ControlState.HOLD_CONTACT:
                if sm.is_hold_complete():
                    held_ms = sm.hold_elapsed_ms or 0.0
                    event = f"HOLD_COMPLETE held={held_ms:.0f}ms"
                    _log(pos_um, force_mN, vel_um_s, event=event)
                    sm.enter_state(ControlState.RETURN_HOME)
                    return_start_s = now
                else:
                    _log(pos_um, force_mN, vel_um_s)

            # ---- RETURN_HOME: wait for shaft to reach home ----
            elif current_state == ControlState.RETURN_HOME:
                if return_start_s is None:
                    return_start_s = now

                if abs(pos_um - home_um) <= home_tol_um:
                    event = "HOME_REACHED"
                    ok = True
                    reason = "CONTACT" if contact_detected else "EMPTY"
                    _log(pos_um, force_mN, vel_um_s, event=event)
                    break

                return_elapsed = now - return_start_s
                if return_elapsed > return_timeout_s:
                    event = f"RETURN_TIMEOUT elapsed={return_elapsed:.2f}s"
                    _log(pos_um, force_mN, vel_um_s, event=event)
                    reason = "RETURN_TIMEOUT"
                    ok = False
                    break

                _log(pos_um, force_mN, vel_um_s)

            time.sleep(dt)

    except KeyboardInterrupt:
        ok = False
        reason = "INTERRUPTED"

    except OrcaDeviceError as ex:
        ok = False
        reason = f"ORCA_ERROR: {ex}"

    except Exception as ex:
        ok = False
        reason = f"ERROR: {ex}"

    finally:
        csv_path = str(logger.path) if logger.path else None
        logger.close()

        # Best-effort safe return if cycle ended abnormally
        if not ok:
            sm.safe_return()

    return ApplyV2CycleResult(
        ok=ok,
        reason=reason,
        contact_detected=contact_detected,
        apply_force_mN=la.apply_force_mN,
        home_um=home_um,
        spring_b_position_um=spring_b_position_um,
        elapsed_s=time.time() - t0,
        csv_path=csv_path,
    )