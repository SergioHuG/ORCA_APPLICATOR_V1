# src/orca_v1/config.py
"""
config.py (V1)

- Typed configuration schema for the Orca applicator V1 repo
- YAML loader for configs/default.yaml
- Minimal validation + sane defaults
- Designed to mirror your current V1 script constants

GPIO refactor:
- Nested per-button configs (autozero vs apply)
- backend + pigpio_host included
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Any, Dict
import os

import yaml


# -----------------------------
# Schema
# -----------------------------
@dataclass(frozen=True)
class SerialConfig:
    port: str = "/dev/orca0"
    baudrate: int = 1_000_000
    interframe_us: int = 80


@dataclass(frozen=True)
class LoopConfig:
    # High-speed loop target rate (stream_cache; non-blocking reads)
    control_hz: int = 250


@dataclass(frozen=True)
class MotionConfig:
    # HOME is computed as (pos_after_autozero + home_offset_um)
    home_offset_um: int = 15_000

    # Motion IDs
    motion_home_id: int = 0
    motion_return_id: int = 1
    motion_apply_id: int = 2

    # Kinematic type: 1=min jerk (as in your script)
    kin_type_min_jerk: int = 1

    # Timing
    motion_home_time_ms: int = 1_000
    motion_return_time_ms: int = 300
    apply_time_ms: int = 500

    # Travel
    apply_travel_um: int = 200_000

    # Switch to haptics before hard apply limit
    haptic_before_limit_um: int = 150_000


@dataclass(frozen=True)
class ContactDetectConfig:
    # Contact detection based on raw force rise from baseline (mN)
    contact_rise_mN: int = 4_000


@dataclass(frozen=True)
class HapticsConfig:
    # Spring effect
    spring_gain: int = 200
    spring_dead_zone: int = 0
    spring_sat_n: int = 10

    # Damper effect
    damper_gain: int = 500


@dataclass(frozen=True)
class LabelApplicatorConfig:
    """
    Configuration for the technical-note label applicator (V2 haptic-only architecture).

    All three springs plus the damper are configured once after AutoZero, then left
    running for the entire session.  Runtime state changes are made exclusively via
    set_constant_force() — no kinematic motions are used after AutoZero.

    Position fields marked "(offset from home_um)" are added to the AutoZero-derived
    home position at runtime to produce absolute motor positions.

    apply_force_mN is computed automatically:
        apply_force_mN = target_contact_force_mN - gravity_mN
    Do not set it directly in YAML; edit target_contact_force_mN and gravity_mN instead.
    """

    # --- Force parameters ---
    # target_contact_force_mN: the total downward force desired at the label surface.
    # gravity_mN: gravitational force component (shaft mass * g).  Shaft-down orientation
    #   means gravity already contributes to the downward stroke, so the motor only needs
    #   to supply the remainder.
    # apply_force_mN: computed in load_config() as target - gravity.  Written to
    #   set_constant_force() to trigger the apply stroke.
    target_contact_force_mN: int = 20_000
    gravity_mN: int = 20_000
    apply_force_mN: int = 0            # computed; do not set in YAML

    # --- Position offsets (from home_um, computed at AutoZero time) ---
    # extended_position_offset_um: distance from home to the apply/label target.
    #   Spring B is centered here.  Also the position threshold used in contact detection
    #   (shaft must be before this point for contact to be valid).
    # spring_c_position_offset_um: distance from home to the Spring C center.
    #   Spring C provides the slingshot acceleration through the fast-travel zone.
    #   Must be strictly between 0 and extended_position_offset_um.
    extended_position_offset_um: int = 180_000
    spring_c_position_offset_um: int = 90_000

    # --- Spring A — home bumper ---
    # Centered at home_um, coupling=both.  Holds the shaft at rest and provides the
    # restoring force for the return stroke when constant_force is set to 0.
    spring_a_gain: int = 500
    spring_a_dead_zone: int = 0
    spring_a_saturation_mN: int = 60_000

    # --- Spring B — end-of-travel bumper ---
    # Centered at home_um + extended_position_offset_um, coupling=both.
    # Acts as a hard stop guard at the far end of travel.
    spring_b_gain: int = 500
    spring_b_dead_zone: int = 0
    spring_b_saturation_mN: int = 60_000

    # --- Spring C — slingshot / fast-travel assist ---
    # Centered at home_um + spring_c_position_offset_um, coupling=positive only.
    # Adds a forward-only push through the fast-travel zone; does not resist retraction.
    spring_c_gain: int = 200
    spring_c_dead_zone: int = 0
    spring_c_saturation_mN: int = 30_000

    # --- Damper ---
    # Velocity-proportional braking applied across the full stroke.  Controls approach
    # speed and reduces bounce at contact.
    damper_gain: int = 300

    # --- Contact detection (three-condition check from technical note) ---
    # Contact is triggered when ALL three conditions hold simultaneously:
    #   1. abs(force - apply_force_mN) <= force_detection_envelope_mN
    #   2. abs(velocity)               <= stopped_velocity_um_s
    #   3. position                    <  (home_um + extended_position_offset_um)
    stopped_velocity_um_s: int = 1_000      # um/s; shaft considered stopped below this
    force_detection_envelope_mN: int = 5_000  # mN; tolerance band around apply_force_mN

    # --- State timing ---
    # Duration to hold in HOLD_CONTACT state (constant force still active) before
    # writing set_constant_force(0) to begin the return stroke.
    hold_contact_duration_ms: int = 500


@dataclass(frozen=True)
class TimeoutsConfig:
    autozero_timeout_s: float = 60.0
    home_timeout_s: float = 10.0
    return_timeout_s: float = 10.0


@dataclass(frozen=True)
class TolerancesConfig:
    home_tol_um: int = 800


# ---- GPIO refactor ----
@dataclass(frozen=True)
class GPIOButtonConfig:
    """
    Per-button configuration. Designed to map cleanly from YAML.

    bounce_time_s:
      - None -> no gpiozero debounce (fastest)
      - float -> gpiozero bounce_time seconds

    lockout_ms:
      software lockout applied in our code (recommended even if bounce_time_s is None).
    """
    pin: Optional[int] = None
    pull_up: bool = True
    bounce_time_s: Optional[float] = 0.05
    lockout_ms: int = 200


@dataclass(frozen=True)
class GPIOConfig:
    """
    GPIO configuration (reserved for V1 integration).

    backend: "pigpio" is recommended for timing and venv friendliness.
    pigpio_host: usually "localhost" (requires pigpiod running).
    """
    enabled: bool = False
    backend: str = "pigpio"
    pigpio_host: str = "localhost"

    autozero: GPIOButtonConfig = GPIOButtonConfig()
    apply: GPIOButtonConfig = GPIOButtonConfig(bounce_time_s=None, lockout_ms=150)


@dataclass(frozen=True)
class LoggingConfig:
    log_dir: str = "logs"
    csv_prefix: str = "apply_kin_then_haptic"


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig = SerialConfig()
    loop: LoopConfig = LoopConfig()
    motion: MotionConfig = MotionConfig()
    contact: ContactDetectConfig = ContactDetectConfig()
    haptics: HapticsConfig = HapticsConfig()
    timeouts: TimeoutsConfig = TimeoutsConfig()
    tolerances: TolerancesConfig = TolerancesConfig()
    gpio: GPIOConfig = GPIOConfig()
    logging: LoggingConfig = LoggingConfig()

    # Behavior flags (your new sequencing)
    autozero_home_on_boot: bool = True

    # V2 label applicator config (technical-note haptic-only architecture).
    # None means not configured; V1 code paths never read this field.
    label_applicator: Optional[LabelApplicatorConfig] = None


# -----------------------------
# Loading helpers
# -----------------------------
def _deep_get(d: Dict[str, Any], *keys: str, default: Any = None) -> Any:
    cur: Any = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _env_override_str(name: str) -> Optional[str]:
    val = os.getenv(name)
    return val if val not in (None, "") else None


def _env_override_int(name: str) -> Optional[int]:
    val = _env_override_str(name)
    if val is None:
        return None
    try:
        return int(val)
    except ValueError:
        raise ValueError(f"Env var {name} must be int, got: {val}")


def _env_override_bool(name: str) -> Optional[bool]:
    val = _env_override_str(name)
    if val is None:
        return None
    v = val.strip().lower()
    if v in ("1", "true", "yes", "y", "on"):
        return True
    if v in ("0", "false", "no", "n", "off"):
        return False
    raise ValueError(f"Env var {name} must be boolean-ish, got: {val}")


def load_config(path: str | Path) -> AppConfig:
    """
    Load YAML config and return typed AppConfig.

    Minimal env overrides supported now (optional):
      ORCA_PORT, ORCA_BAUDRATE, ORCA_INTERFRAME_US
      ORCA_CONTROL_HZ
      ORCA_AUTOZERO_ON_BOOT

    (GPIO env overrides can be added later if needed.)
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Config not found: {p}")

    raw = yaml.safe_load(p.read_text()) or {}
    if not isinstance(raw, dict):
        raise ValueError("Config YAML root must be a mapping/object")

    # --- Build typed configs from YAML (with defaults) ---
    serial = SerialConfig(
        port=str(_deep_get(raw, "serial", "port", default=SerialConfig.port)),
        baudrate=int(_deep_get(raw, "serial", "baudrate", default=SerialConfig.baudrate)),
        interframe_us=int(_deep_get(raw, "serial", "interframe_us", default=SerialConfig.interframe_us)),
    )

    loop = LoopConfig(
        control_hz=int(_deep_get(raw, "loop", "control_hz", default=LoopConfig.control_hz)),
    )

    motion = MotionConfig(
        home_offset_um=int(_deep_get(raw, "motion", "home_offset_um", default=MotionConfig.home_offset_um)),
        motion_home_id=int(_deep_get(raw, "motion", "motion_home_id", default=MotionConfig.motion_home_id)),
        motion_return_id=int(_deep_get(raw, "motion", "motion_return_id", default=MotionConfig.motion_return_id)),
        motion_apply_id=int(_deep_get(raw, "motion", "motion_apply_id", default=MotionConfig.motion_apply_id)),
        kin_type_min_jerk=int(_deep_get(raw, "motion", "kin_type_min_jerk", default=MotionConfig.kin_type_min_jerk)),
        motion_home_time_ms=int(_deep_get(raw, "motion", "motion_home_time_ms", default=MotionConfig.motion_home_time_ms)),
        motion_return_time_ms=int(_deep_get(raw, "motion", "motion_return_time_ms", default=MotionConfig.motion_return_time_ms)),
        apply_time_ms=int(_deep_get(raw, "motion", "apply_time_ms", default=MotionConfig.apply_time_ms)),
        apply_travel_um=int(_deep_get(raw, "motion", "apply_travel_um", default=MotionConfig.apply_travel_um)),
        haptic_before_limit_um=int(_deep_get(raw, "motion", "haptic_before_limit_um", default=MotionConfig.haptic_before_limit_um)),
    )

    contact = ContactDetectConfig(
        contact_rise_mN=int(_deep_get(raw, "contact", "contact_rise_mN", default=ContactDetectConfig.contact_rise_mN)),
    )

    haptics = HapticsConfig(
        spring_gain=int(_deep_get(raw, "haptics", "spring_gain", default=HapticsConfig.spring_gain)),
        spring_dead_zone=int(_deep_get(raw, "haptics", "spring_dead_zone", default=HapticsConfig.spring_dead_zone)),
        spring_sat_n=int(_deep_get(raw, "haptics", "spring_sat_n", default=HapticsConfig.spring_sat_n)),
        damper_gain=int(_deep_get(raw, "haptics", "damper_gain", default=HapticsConfig.damper_gain)),
    )

    timeouts = TimeoutsConfig(
        autozero_timeout_s=float(_deep_get(raw, "timeouts", "autozero_timeout_s", default=TimeoutsConfig.autozero_timeout_s)),
        home_timeout_s=float(_deep_get(raw, "timeouts", "home_timeout_s", default=TimeoutsConfig.home_timeout_s)),
        return_timeout_s=float(_deep_get(raw, "timeouts", "return_timeout_s", default=TimeoutsConfig.return_timeout_s)),
    )

    tolerances = TolerancesConfig(
        home_tol_um=int(_deep_get(raw, "tolerances", "home_tol_um", default=TolerancesConfig.home_tol_um)),
    )

    # GPIO (nested)
    gpio_backend = str(_deep_get(raw, "gpio", "backend", default=GPIOConfig.backend))
    gpio_host = str(_deep_get(raw, "gpio", "pigpio_host", default=GPIOConfig.pigpio_host))

    autozero_btn = GPIOButtonConfig(
        pin=_deep_get(raw, "gpio", "autozero", "pin", default=GPIOConfig.autozero.pin),
        pull_up=bool(_deep_get(raw, "gpio", "autozero", "pull_up", default=GPIOConfig.autozero.pull_up)),
        bounce_time_s=_deep_get(raw, "gpio", "autozero", "bounce_time_s", default=GPIOConfig.autozero.bounce_time_s),
        lockout_ms=int(_deep_get(raw, "gpio", "autozero", "lockout_ms", default=GPIOConfig.autozero.lockout_ms)),
    )

    apply_btn = GPIOButtonConfig(
        pin=_deep_get(raw, "gpio", "apply", "pin", default=GPIOConfig.apply.pin),
        pull_up=bool(_deep_get(raw, "gpio", "apply", "pull_up", default=GPIOConfig.apply.pull_up)),
        bounce_time_s=_deep_get(raw, "gpio", "apply", "bounce_time_s", default=GPIOConfig.apply.bounce_time_s),
        lockout_ms=int(_deep_get(raw, "gpio", "apply", "lockout_ms", default=GPIOConfig.apply.lockout_ms)),
    )

    gpio = GPIOConfig(
        enabled=bool(_deep_get(raw, "gpio", "enabled", default=GPIOConfig.enabled)),
        backend=gpio_backend,
        pigpio_host=gpio_host,
        autozero=autozero_btn,
        apply=apply_btn,
    )

    logging_cfg = LoggingConfig(
        log_dir=str(_deep_get(raw, "logging", "log_dir", default=LoggingConfig.log_dir)),
        csv_prefix=str(_deep_get(raw, "logging", "csv_prefix", default=LoggingConfig.csv_prefix)),
    )

    autozero_home_on_boot = bool(_deep_get(raw, "behavior", "autozero_home_on_boot", default=True))

    # --- label_applicator (V2 haptic-only architecture; optional section) ---
    # If the YAML contains a `label_applicator:` key, load it fully.
    # If the key is absent, leave label_applicator=None so V1 code paths are unaffected.
    label_applicator: Optional[LabelApplicatorConfig] = None
    if "label_applicator" in raw:
        la = raw["label_applicator"] if isinstance(raw["label_applicator"], dict) else {}

        target_contact_force_mN = int(_deep_get(la, "target_contact_force_mN",
            default=LabelApplicatorConfig.target_contact_force_mN))
        gravity_mN = int(_deep_get(la, "gravity_mN",
            default=LabelApplicatorConfig.gravity_mN))

        label_applicator = LabelApplicatorConfig(
            target_contact_force_mN=target_contact_force_mN,
            gravity_mN=gravity_mN,
            apply_force_mN=target_contact_force_mN - gravity_mN,  # computed

            extended_position_offset_um=int(_deep_get(la, "extended_position_offset_um",
                default=LabelApplicatorConfig.extended_position_offset_um)),
            spring_c_position_offset_um=int(_deep_get(la, "spring_c_position_offset_um",
                default=LabelApplicatorConfig.spring_c_position_offset_um)),

            spring_a_gain=int(_deep_get(la, "spring_a_gain",
                default=LabelApplicatorConfig.spring_a_gain)),
            spring_a_dead_zone=int(_deep_get(la, "spring_a_dead_zone",
                default=LabelApplicatorConfig.spring_a_dead_zone)),
            spring_a_saturation_mN=int(_deep_get(la, "spring_a_saturation_mN",
                default=LabelApplicatorConfig.spring_a_saturation_mN)),

            spring_b_gain=int(_deep_get(la, "spring_b_gain",
                default=LabelApplicatorConfig.spring_b_gain)),
            spring_b_dead_zone=int(_deep_get(la, "spring_b_dead_zone",
                default=LabelApplicatorConfig.spring_b_dead_zone)),
            spring_b_saturation_mN=int(_deep_get(la, "spring_b_saturation_mN",
                default=LabelApplicatorConfig.spring_b_saturation_mN)),

            spring_c_gain=int(_deep_get(la, "spring_c_gain",
                default=LabelApplicatorConfig.spring_c_gain)),
            spring_c_dead_zone=int(_deep_get(la, "spring_c_dead_zone",
                default=LabelApplicatorConfig.spring_c_dead_zone)),
            spring_c_saturation_mN=int(_deep_get(la, "spring_c_saturation_mN",
                default=LabelApplicatorConfig.spring_c_saturation_mN)),

            damper_gain=int(_deep_get(la, "damper_gain",
                default=LabelApplicatorConfig.damper_gain)),

            stopped_velocity_um_s=int(_deep_get(la, "stopped_velocity_um_s",
                default=LabelApplicatorConfig.stopped_velocity_um_s)),
            force_detection_envelope_mN=int(_deep_get(la, "force_detection_envelope_mN",
                default=LabelApplicatorConfig.force_detection_envelope_mN)),

            hold_contact_duration_ms=int(_deep_get(la, "hold_contact_duration_ms",
                default=LabelApplicatorConfig.hold_contact_duration_ms)),
        )

    cfg = AppConfig(
        serial=serial,
        loop=loop,
        motion=motion,
        contact=contact,
        haptics=haptics,
        timeouts=timeouts,
        tolerances=tolerances,
        gpio=gpio,
        logging=logging_cfg,
        autozero_home_on_boot=autozero_home_on_boot,
        label_applicator=label_applicator,
    )

    # --- Apply minimal env overrides ---
    port = _env_override_str("ORCA_PORT")
    baud = _env_override_int("ORCA_BAUDRATE")
    if_us = _env_override_int("ORCA_INTERFRAME_US")
    hz = _env_override_int("ORCA_CONTROL_HZ")
    az_boot = _env_override_bool("ORCA_AUTOZERO_ON_BOOT")

    if port is not None or baud is not None or if_us is not None:
        cfg = AppConfig(
            **{**cfg.__dict__, "serial": SerialConfig(
                port=port if port is not None else cfg.serial.port,
                baudrate=baud if baud is not None else cfg.serial.baudrate,
                interframe_us=if_us if if_us is not None else cfg.serial.interframe_us,
            )}
        )

    if hz is not None:
        cfg = AppConfig(**{**cfg.__dict__, "loop": LoopConfig(control_hz=hz)})

    if az_boot is not None:
        cfg = AppConfig(**{**cfg.__dict__, "autozero_home_on_boot": az_boot})

    _validate(cfg)
    return cfg


def _validate(cfg: AppConfig) -> None:
    # Basic checks (V1)
    if cfg.loop.control_hz <= 0 or cfg.loop.control_hz > 2000:
        raise ValueError(f"loop.control_hz out of range: {cfg.loop.control_hz}")

    if cfg.serial.baudrate <= 0:
        raise ValueError(f"serial.baudrate must be > 0: {cfg.serial.baudrate}")

    if cfg.motion.apply_travel_um <= 0:
        raise ValueError("motion.apply_travel_um must be > 0")

    if cfg.motion.haptic_before_limit_um < 0 or cfg.motion.haptic_before_limit_um > cfg.motion.apply_travel_um:
        raise ValueError("motion.haptic_before_limit_um must be within [0, apply_travel_um]")

    if cfg.contact.contact_rise_mN <= 0:
        raise ValueError("contact.contact_rise_mN must be > 0")

    if cfg.haptics.spring_gain < 0 or cfg.haptics.damper_gain < 0:
        raise ValueError("haptics gains must be >= 0")

    if cfg.timeouts.autozero_timeout_s <= 0:
        raise ValueError("timeouts.autozero_timeout_s must be > 0")

    # GPIO checks
    if cfg.gpio.enabled:
        if cfg.gpio.backend not in ("pigpio",):
            raise ValueError(f"gpio.backend unsupported in V1: {cfg.gpio.backend}")

        # Pins are allowed to be None for now if you want to run without a given trigger,
        # but at least one should be configured if gpio is enabled.
        if cfg.gpio.autozero.pin is None and cfg.gpio.apply.pin is None:
            raise ValueError("gpio.enabled is true but both gpio.autozero.pin and gpio.apply.pin are null")

        for name, btn in (("autozero", cfg.gpio.autozero), ("apply", cfg.gpio.apply)):
            if btn.pin is not None and (btn.pin < 0 or btn.pin > 27):
                # BCM pins on Pi typically 0..27 in common use (not strict, but good guard)
                raise ValueError(f"gpio.{name}.pin out of expected BCM range: {btn.pin}")

            if btn.lockout_ms < 0:
                raise ValueError(f"gpio.{name}.lockout_ms must be >= 0")

            if btn.bounce_time_s is not None and btn.bounce_time_s < 0:
                raise ValueError(f"gpio.{name}.bounce_time_s must be null or >= 0")

    # --- label_applicator checks (only when section is present) ---
    if cfg.label_applicator is not None:
        la = cfg.label_applicator

        if la.apply_force_mN <= 0:
            raise ValueError(
                f"label_applicator: apply_force_mN must be > 0 "
                f"(got target={la.target_contact_force_mN} - gravity={la.gravity_mN} "
                f"= {la.apply_force_mN}). Increase target_contact_force_mN or reduce gravity_mN."
            )

        if la.extended_position_offset_um <= 0:
            raise ValueError("label_applicator.extended_position_offset_um must be > 0")

        if not (0 < la.spring_c_position_offset_um < la.extended_position_offset_um):
            raise ValueError(
                f"label_applicator.spring_c_position_offset_um ({la.spring_c_position_offset_um}) "
                f"must be strictly between 0 and extended_position_offset_um "
                f"({la.extended_position_offset_um})"
            )

        for name, gain, sat in (
            ("spring_a", la.spring_a_gain, la.spring_a_saturation_mN),
            ("spring_b", la.spring_b_gain, la.spring_b_saturation_mN),
            ("spring_c", la.spring_c_gain, la.spring_c_saturation_mN),
        ):
            if gain < 0:
                raise ValueError(f"label_applicator.{name}_gain must be >= 0")
            if sat <= 0:
                raise ValueError(f"label_applicator.{name}_saturation_mN must be > 0")

        if la.damper_gain < 0:
            raise ValueError("label_applicator.damper_gain must be >= 0")

        if la.stopped_velocity_um_s <= 0:
            raise ValueError("label_applicator.stopped_velocity_um_s must be > 0")

        if la.force_detection_envelope_mN <= 0:
            raise ValueError("label_applicator.force_detection_envelope_mN must be > 0")

        if la.hold_contact_duration_ms < 0:
            raise ValueError("label_applicator.hold_contact_duration_ms must be >= 0")