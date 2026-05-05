# src/orca_v1/haptics.py
"""
haptics.py (V1)

Haptic "soft bumper" setup and upkeep:
- Switch to HapticMode
- Enable Spring0 + Damper effects
- Configure spring (center position, gain, dead zone, saturation, coupling)
- Configure damper gain
- Update stream effects each loop tick (keeps effects active)

This mirrors your current testing script behavior, just packaged cleanly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from pyorcasdk import MotorMode, HapticEffect, SpringCoupling

from .orca_client import OrcaClient


@dataclass(frozen=True)
class SoftBumperConfig:
    # Spring effect
    spring_gain: int
    spring_dead_zone: int
    spring_sat_n: int
    # Damper effect
    damper_gain: int

    # Spring index (we use Spring0 like your script)
    spring_index: int = 0

    # Spring coupling direction (positive like your script)
    spring_coupling: int = int(SpringCoupling.positive)


def effects_mask_soft_bumper() -> int:
    """Bitmask for Spring0 + Damper."""
    return int(HapticEffect.Spring0 | HapticEffect.Damper)


def enter_soft_bumper(
    client: OrcaClient,
    *,
    cfg: SoftBumperConfig,
    center_um: int,
) -> None:
    """
    Enter HapticMode and configure spring+damper.

    center_um:
      The spring center position. In V1 we typically use the current position
      at the moment we enter haptics (matches your test script).
    """
    motor = client.motor

    # 1) Enter HapticMode
    client.set_mode(MotorMode.HapticMode)

    # 2) Enable and update effects (mask)
    mask = effects_mask_soft_bumper()

    err = motor.enable_haptic_effects(mask)
    if err:
        raise RuntimeError(f"enable_haptic_effects failed: {err.what()}")

    err = motor.update_haptic_stream_effects(mask)
    if err:
        raise RuntimeError(f"update_haptic_stream_effects failed: {err.what()}")

    # 3) Configure Spring0
    # Signature based on your working script:
    # set_spring_effect(index, gain, center, dead_zone, sat_n, coupling)
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

    # 4) Configure Damper gain
    err = motor.set_damper(int(cfg.damper_gain))
    if err:
        raise RuntimeError(f"set_damper failed: {err.what()}")


def keep_soft_bumper_alive(client: OrcaClient) -> None:
    """
    Call this during the high-speed loop while in haptics.
    This matches your script's periodic update_haptic_stream_effects call.
    """
    motor = client.motor
    mask = effects_mask_soft_bumper()
    motor.update_haptic_stream_effects(mask)


def build_soft_bumper_config_from_app_config(app_cfg) -> SoftBumperConfig:
    """
    Convenience adapter from AppConfig -> SoftBumperConfig.

    We avoid importing AppConfig directly here to keep V1 modules loosely coupled.
    """
    return SoftBumperConfig(
        spring_gain=int(app_cfg.haptics.spring_gain),
        spring_dead_zone=int(app_cfg.haptics.spring_dead_zone),
        spring_sat_n=int(app_cfg.haptics.spring_sat_n),
        damper_gain=int(app_cfg.haptics.damper_gain),
    )
