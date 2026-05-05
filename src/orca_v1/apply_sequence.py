# src/orca_v1/apply_sequence.py
"""
apply_sequence.py (V1)

Implements the core motion sequence using stream_cache-first reads:

(AutoZero -> Home) -> Idle (handled by main.py) -> Kinematic Apply -> Haptic Soft Bumper
-> Contact (baseline+delta) -> Return Home -> back to IDLE

This module focuses on the Orca sequence + logging. GPIO is handled elsewhere.

Key constraints:
- High-speed loop avoids blocking Modbus reads. Uses:
    enable_stream() + run() + get_stream_data() (cache)
- Safety > speed: explicit timeouts, safe exit to return home on anomalies.

Public API:
- program_motions(...)
- autozero_and_home(...)
- run_apply_cycle(...): does APPLY->HAPTIC->CONTACT/RETURN and returns a result
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import time

from pyorcasdk import MotorMode

from .config import AppConfig
from .orca_client import OrcaClient, OrcaDeviceError
from .logging_csv import CsvLogger, CsvLogConfig
from .haptics import enter_soft_bumper, keep_soft_bumper_alive, build_soft_bumper_config_from_app_config
from .contact_detect import build_contact_detector_from_app_config


# -----------------------------
# Result types
# -----------------------------
@dataclass(frozen=True)
class ApplyCycleResult:
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


# -----------------------------
# Helpers
# -----------------------------
def _wait_reach_position(
    client: OrcaClient,
    target_um: int,
    tol_um: int,
    timeout_s: float,
    poll_s: float = 0.02,
) -> Optional[int]:
    """
    Blocking-ish wait using cached stream if available; falls back to blocking read if needed.
    This is not the high-speed portion; it's for HOME/RETURN completion checks.
    """
    t0 = time.time()
    while (time.time() - t0) < timeout_s:
        # Prefer stream cache (non-blocking). Ensure run() is called.
        try:
            client.run()
            snap = client.read_stream_cache()
            pos = snap.position_um
        except Exception:
            pos = None

        if pos is None:
            # fallback to a blocking read if stream isn't healthy
            try:
                pos = client.get_position_um_blocking()
            except Exception:
                pos = None

        if pos is not None and abs(int(pos) - int(target_um)) <= int(tol_um):
            return int(pos)

        time.sleep(poll_s)

    return None


def program_motions(client: OrcaClient, cfg: AppConfig, home_um: int) -> None:
    """
    Program kinematic motions:
      - Motion HOME (id=motion_home_id)
      - Motion RETURN (id=motion_return_id)
      - Motion APPLY (id=motion_apply_id)
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
    AutoZero, compute HOME, program motions, and move to HOME.
    Returns home_um.
    """
    # AutoZero
    client.set_mode(MotorMode.AutoZeroMode)
    client.wait_until_not_mode(MotorMode.AutoZeroMode, timeout_s=cfg.timeouts.autozero_timeout_s)

    pos_after_um = client.get_position_um_blocking()
    home_um = pos_after_um + cfg.motion.home_offset_um

    # Program kinematic motions (HOME/RETURN/APPLY)
    program_motions(client, cfg, home_um=home_um)

    # Enter KinematicMode and go HOME
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
        # Not fatal for V1; caller can decide.
        print("[AUTOZERO_HOME] Warning: HOME not reached within tolerance/time (continuing).")
    else:
        print(f"[AUTOZERO_HOME] HOME reached. pos={reached}um")

    return home_um


# -----------------------------
# Apply cycle
# -----------------------------
def run_apply_cycle(client: OrcaClient, cfg: AppConfig, home_um: int) -> ApplyCycleResult:
    """
    Execute:
      Kinematic APPLY -> enter HAPTIC bumper at entry point -> detect CONTACT
      -> trigger RETURN -> wait HOME reached (within tolerance)
    """
    m = cfg.motion
    control_hz = cfg.loop.control_hz
    dt = 1.0 / float(control_hz)

    apply_target_um = home_um + m.apply_travel_um
    haptic_entry_um = home_um + (m.apply_travel_um - m.haptic_before_limit_um)

    # Ensure kinematic mode before apply
    client.set_mode(MotorMode.KinematicMode)

    # Trigger APPLY motion
    client.trigger_kinematic_motion(m.motion_apply_id)

    # Warmup stream cache (non-fatal)
    alive = client.wait_for_stream_alive(timeout_s=2.0)
    if not alive:
        print("[APPLY] Warning: stream_cache did not show updates during warmup window.")

    # Prepare logging
    csv_cfg = CsvLogConfig(
        log_dir=cfg.logging.log_dir,
        prefix=cfg.logging.csv_prefix,
        include_header=True,
    )
    logger = CsvLogger(csv_cfg).open()

    # Contact detector
    det = build_contact_detector_from_app_config(cfg)

    # Haptics config
    bumper_cfg = build_soft_bumper_config_from_app_config(cfg)

    # State
    t0 = time.time()
    last_t = t0
    last_pos: Optional[int] = None

    phase = "APPLY_KIN"
    haptic_entered = False
    baseline_mN: Optional[int] = None
    peak_force = -10**9
    contact_delta: Optional[int] = None

    reason = "UNKNOWN"
    ok = False

    try:
        while True:
            now = time.time()
            elapsed = now - t0

            # Keep comms alive
            client.run()

            # Non-blocking stream cache read
            snap = client.read_stream_cache()
            pos_um = snap.position_um
            force_mN = snap.force_mN
            peak_force = max(peak_force, force_mN)

            # If stream isn't ready, log and continue
            if client.is_stream_stale(snap):
                logger.log(
                    epoch_s=now,
                    phase=phase,
                    elapsed_s=elapsed,
                    position_um=pos_um,
                    velocity_um_s=0.0,
                    force_raw_mN=force_mN,
                    baseline_mN=baseline_mN,
                    event="STREAM_NOT_READY",
                )
                time.sleep(dt)
                continue

            # Velocity estimate
            vel = 0.0
            if last_pos is not None:
                dtt = max(1e-6, now - last_t)
                vel = (pos_um - last_pos) / dtt
            last_pos = pos_um
            last_t = now

            event = ""

            # Step A: enter haptics at entry point
            if (not haptic_entered) and (pos_um >= haptic_entry_um):
                phase = "HAPTIC_BUMPER"
                enter_soft_bumper(client, cfg=bumper_cfg, center_um=pos_um)

                baseline_mN = force_mN
                det.capture_baseline(force_mN, now)
                haptic_entered = True
                event = f"ENTER_HAPTIC baseline={baseline_mN}"

            # Step B: in haptics, keep effects alive and check for contact
            if phase == "HAPTIC_BUMPER":
                keep_soft_bumper_alive(client)

                res = det.update(force_mN, now)
                if res.triggered:
                    event = "CONTACT_TRIGGER"
                    contact_delta = res.delta_mN

                    # Return home
                    client.set_mode(MotorMode.KinematicMode)
                    client.trigger_kinematic_motion(m.motion_return_id)
                    phase = "RETURN_HOME"

            # Step C: fail-safe if we reached apply target without contact
            if phase in ("APPLY_KIN", "HAPTIC_BUMPER"):
                if pos_um >= (apply_target_um - 500):
                    event = "APPLY_REACHED_TARGET"
                    client.set_mode(MotorMode.KinematicMode)
                    client.trigger_kinematic_motion(m.motion_return_id)
                    phase = "RETURN_HOME"

            # Step D: complete when home reached
            if phase == "RETURN_HOME":
                if abs(pos_um - home_um) <= cfg.tolerances.home_tol_um:
                    event = (event + "|" if event else "") + "HOME_REACHED"
                    ok = True
                    reason = "COMPLETED"
                    logger.log(
                        epoch_s=now,
                        phase=phase,
                        elapsed_s=elapsed,
                        position_um=pos_um,
                        velocity_um_s=vel,
                        force_raw_mN=force_mN,
                        baseline_mN=baseline_mN,
                        event=event,
                    )
                    break

                # Timeout for return (safety)
                if elapsed > (cfg.timeouts.return_timeout_s + 30.0):
                    # generous additional window; adjust later once tuned
                    reason = "RETURN_TIMEOUT"
                    ok = False
                    break

            # Log tick
            logger.log(
                epoch_s=now,
                phase=phase,
                elapsed_s=elapsed,
                position_um=pos_um,
                velocity_um_s=vel,
                force_raw_mN=force_mN,
                baseline_mN=baseline_mN,
                event=event,
            )

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

        # Best-effort: if we exited mid-cycle and are not in IDLE, try to return safely
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
        peak_force_mN=peak_force if peak_force != -10**9 else 0,
        contact_delta_mN=contact_delta,
        elapsed_s=time.time() - t0,
        csv_path=csv_path,
    )
