# src/orca_v1/haptics.py
"""
haptics.py

Two architectures live here side-by-side:

V1 — soft bumper (single Spring0 + Damper, configured live at haptic-entry point):
    SoftBumperConfig, enter_soft_bumper(), keep_soft_bumper_alive(),
    effects_mask_soft_bumper(), build_soft_bumper_config_from_app_config()

V2 — technical-note haptic-only architecture (three springs + Damper, configured
once after AutoZero with fixed absolute positions, never reconfigured at runtime):
    configure_haptic_effects(), keep_haptic_effects_alive()

V1 functions are preserved unchanged so the V1 apply cycle continues to work.
V2 functions are additive and do not touch any V1 code path.

HapticEffect bit values (pyorcasdk 1.1.0):
    Spring0=2  Spring1=4  Spring2=8  Damper=16
SpringCoupling values:
    both=0  positive=1  negative=2
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from pyorcasdk import MotorMode, HapticEffect, SpringCoupling

from .orca_client import OrcaClient
from .config import LabelApplicatorConfig


# ============================================================
# V1 — Soft bumper (single spring, configured at haptic entry)
# ============================================================

@dataclass(frozen=True)
class SoftBumperConfig:
    # Spring effect
    spring_gain: int
    spring_dead_zone: int
    spring_sat_n: int
    # Damper effect
    damper_gain: int

    # Spring index (V1 uses Spring0)
    spring_index: int = 0

    # Spring coupling direction
    spring_coupling: int = int(SpringCoupling.positive)


def effects_mask_soft_bumper() -> int:
    """Bitmask for V1 soft bumper: Spring0 + Damper."""
    return int(HapticEffect.Spring0 | HapticEffect.Damper)


def enter_soft_bumper(
    client: OrcaClient,
    *,
    cfg: SoftBumperConfig,
    center_um: int,
) -> None:
    """
    Enter HapticMode and configure a single spring + damper (V1 soft bumper).

    Called once per cycle at the haptic-entry position. The spring center is
    set to whatever position the shaft occupies at that moment (center_um).

    V2 does not use this function. V2 calls configure_haptic_effects() once
    at boot, never again.
    """
    motor = client.motor

    # Enter HapticMode first (V1 sequencing)
    client.set_mode(MotorMode.HapticMode)

    mask = effects_mask_soft_bumper()

    err = motor.enable_haptic_effects(mask)
    if err:
        raise RuntimeError(f"enable_haptic_effects failed: {err.what()}")

    err = motor.update_haptic_stream_effects(mask)
    if err:
        raise RuntimeError(f"update_haptic_stream_effects failed: {err.what()}")

    err = motor.set_spring_effect(
        int(cfg.spring_index),
        int(cfg.spring_gain),
        int(center_um),
        int(cfg.spring_dead_zone),
        int(cfg.spring_sat_n),
        int(cfg.spring_coupling),
    )
    if err:
        raise RuntimeError(f"set_spring_effect failed: {err.what()}")

    err = motor.set_damper(int(cfg.damper_gain))
    if err:
        raise RuntimeError(f"set_damper failed: {err.what()}")


def keep_soft_bumper_alive(client: OrcaClient) -> None:
    """
    Call each tick while in V1 haptics to keep stream effects active.
    Not used by V2.
    """
    motor = client.motor
    mask = effects_mask_soft_bumper()
    motor.update_haptic_stream_effects(mask)


def build_soft_bumper_config_from_app_config(app_cfg) -> SoftBumperConfig:
    """Adapter from AppConfig -> SoftBumperConfig (V1 only)."""
    return SoftBumperConfig(
        spring_gain=int(app_cfg.haptics.spring_gain),
        spring_dead_zone=int(app_cfg.haptics.spring_dead_zone),
        spring_sat_n=int(app_cfg.haptics.spring_sat_n),
        damper_gain=int(app_cfg.haptics.damper_gain),
    )


# ============================================================
# V2 — Three-spring haptic architecture (technical note)
# ============================================================

# Spring slot assignments (fixed by architecture, not configurable)
_SPRING_ID_A = 0   # Spring0 — home bumper, coupling=both
_SPRING_ID_B = 1   # Spring1 — end-of-travel bumper, coupling=both
_SPRING_ID_C = 2   # Spring2 — slingshot / fast-travel, coupling=positive

# Bitmask covering all four V2 effects
# Spring0(2) | Spring1(4) | Spring2(8) | Damper(16) = 30
_V2_EFFECTS_MASK: int = (
    int(HapticEffect.Spring0)
    | int(HapticEffect.Spring1)
    | int(HapticEffect.Spring2)
    | int(HapticEffect.Damper)
)


def configure_haptic_effects(
    client: OrcaClient,
    la_cfg: LabelApplicatorConfig,
    home_um: int,
) -> None:
    """
    Configure and activate all four V2 haptic effects in a single setup call.

    Must be called once after autozero_and_home() returns, before the first
    apply cycle.  The motor stays in HapticMode for the remainder of the
    session — this function is never called again.

    Sequencing (matches technical note guarantee):
        1. Configure each effect (spring A, B, C, then damper)
        2. enable_haptic_effects(mask)
        3. update_haptic_stream_effects(mask)
        4. set_mode(HapticMode)   ← mode transition last

    Spring positions (all absolute, derived from home_um):
        Spring A center = home_um                              (home bumper)
        Spring B center = home_um + extended_position_offset   (end-of-travel)
        Spring C center = home_um + spring_c_position_offset   (slingshot)

    Args:
        client:   OrcaClient with an open serial port.
        la_cfg:   Loaded LabelApplicatorConfig from AppConfig.label_applicator.
        home_um:  Absolute home position returned by autozero_and_home().

    Raises:
        RuntimeError: if any SDK call reports an error.
    """
    motor = client.motor

    # Compute absolute spring positions
    spring_b_um = home_um + la_cfg.extended_position_offset_um
    spring_c_um = home_um + la_cfg.spring_c_position_offset_um

    # --- Step 1: configure effects before enabling ---

    # Spring A — home bumper
    # coupling=both: resists motion in both directions from home.
    # Holds shaft at rest; provides the return restoring force when
    # set_constant_force(0) is written.
    err = motor.set_spring_effect(
        _SPRING_ID_A,
        int(la_cfg.spring_a_gain),
        int(home_um),
        int(la_cfg.spring_a_dead_zone),
        int(la_cfg.spring_a_saturation_mN),
        int(SpringCoupling.both),
    )
    if err:
        raise RuntimeError(f"configure_haptic_effects: set_spring_effect(A) failed: {err.what()}")

    # Spring B — end-of-travel guard
    # coupling=both: hard stop in both directions at the extended position.
    # Prevents shaft over-travel if apply_force_mN is set too high.
    err = motor.set_spring_effect(
        _SPRING_ID_B,
        int(la_cfg.spring_b_gain),
        int(spring_b_um),
        int(la_cfg.spring_b_dead_zone),
        int(la_cfg.spring_b_saturation_mN),
        int(SpringCoupling.both),
    )
    if err:
        raise RuntimeError(f"configure_haptic_effects: set_spring_effect(B) failed: {err.what()}")

    # Spring C — slingshot / fast-travel assist
    # coupling=positive: only pushes in the extension (downward) direction.
    # Does not resist retraction, so Spring A can pull the shaft home cleanly.
    err = motor.set_spring_effect(
        _SPRING_ID_C,
        int(la_cfg.spring_c_gain),
        int(spring_c_um),
        int(la_cfg.spring_c_dead_zone),
        int(la_cfg.spring_c_saturation_mN),
        int(SpringCoupling.positive),
    )
    if err:
        raise RuntimeError(f"configure_haptic_effects: set_spring_effect(C) failed: {err.what()}")

    # Damper — velocity-proportional braking across the full stroke
    err = motor.set_damper(int(la_cfg.damper_gain))
    if err:
        raise RuntimeError(f"configure_haptic_effects: set_damper failed: {err.what()}")

    # --- Step 2: enable effects ---
    err = motor.enable_haptic_effects(_V2_EFFECTS_MASK)
    if err:
        raise RuntimeError(f"configure_haptic_effects: enable_haptic_effects failed: {err.what()}")

    # --- Step 3: push effects into stream ---
    err = motor.update_haptic_stream_effects(_V2_EFFECTS_MASK)
    if err:
        raise RuntimeError(f"configure_haptic_effects: update_haptic_stream_effects failed: {err.what()}")

    # --- Step 4: transition to HapticMode (always last) ---
    client.set_mode(MotorMode.HapticMode)


def keep_haptic_effects_alive(client: OrcaClient) -> None:
    """
    Call each tick of the V2 control loop to keep stream effects active.

    Sends update_haptic_stream_effects() with the full V2 mask.
    Must be called regularly while in HapticMode; failure to do so causes
    the motor to disengage haptic effects after a watchdog timeout.
    """
    client.motor.update_haptic_stream_effects(_V2_EFFECTS_MASK)