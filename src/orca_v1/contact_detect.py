# src/orca_v1/contact_detect.py
"""
contact_detect.py

Two detection architectures live here side-by-side:

V1 — baseline + delta (single force condition):
    ContactDetectConfig, ContactState, ContactResult, ContactDetector,
    build_contact_detector_from_app_config()

V2 — technical-note three-condition check:
    ThreeConditionResult, ThreeConditionContactDetector,
    EmptyResult, EmptyDetector,
    build_v2_contact_detector(), build_v2_empty_detector()

Both architectures are pure Python with no SDK calls and are unit-testable
without hardware.

V2 contact detection triggers when ALL three conditions hold simultaneously:
    1. abs(force_mN - apply_force_mN)  <= force_detection_envelope_mN
    2. abs(velocity_um_s)              <= stopped_velocity_um_s
    3. position_um                     <  spring_b_position_um

V2 empty detection triggers when the shaft reaches the extended position
with no label present (no contact was detected during the apply stroke):
    1. position_um  >= spring_b_position_um
    2. abs(velocity_um_s) <= stopped_velocity_um_s
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# ============================================================
# V1 — Baseline + delta contact detection
# ============================================================

@dataclass(frozen=True)
class ContactDetectConfig:
    # Trigger threshold (mN) applied to delta from baseline
    rise_threshold_mN: int

    # Ignore detection for a short time after baseline capture (ms)
    holdoff_ms: int = 0

    # Require N consecutive samples above threshold (debounce)
    consecutive_hits: int = 1

    # Optional maximum reasonable delta (sanity guard); 0 disables
    max_delta_mN: int = 0


@dataclass(frozen=True)
class ContactState:
    baseline_mN: Optional[int] = None
    baseline_epoch_s: Optional[float] = None
    hits: int = 0


@dataclass(frozen=True)
class ContactResult:
    triggered: bool
    baseline_mN: Optional[int]
    delta_mN: Optional[int]
    hits: int
    reason: str = ""  # e.g. "THRESHOLD", "HOLDOFF", "NO_BASELINE", "SANITY_CLAMP"


class ContactDetector:
    """
    V1 contact detector: triggers when force rises above a threshold
    relative to a captured baseline.

    Usage pattern:
        det = ContactDetector(ContactDetectConfig(rise_threshold_mN=4000))
        det.capture_baseline(force_mN, epoch_s)

        # each tick while in HAPTIC:
        res = det.update(force_mN, epoch_s)
        if res.triggered:
            ...

    Call reset() to reuse across cycles.
    """

    def __init__(self, cfg: ContactDetectConfig):
        if cfg.rise_threshold_mN <= 0:
            raise ValueError("rise_threshold_mN must be > 0")
        if cfg.consecutive_hits <= 0:
            raise ValueError("consecutive_hits must be >= 1")
        if cfg.holdoff_ms < 0:
            raise ValueError("holdoff_ms must be >= 0")
        if cfg.max_delta_mN < 0:
            raise ValueError("max_delta_mN must be >= 0")

        self._cfg = cfg
        self._state = ContactState()

    @property
    def state(self) -> ContactState:
        return self._state

    def reset(self) -> None:
        self._state = ContactState()

    def capture_baseline(self, force_mN: int, epoch_s: float) -> None:
        """Capture baseline force for delta computation. Resets hit counter."""
        self._state = ContactState(
            baseline_mN=int(force_mN),
            baseline_epoch_s=float(epoch_s),
            hits=0,
        )

    def update(self, force_mN: int, epoch_s: float) -> ContactResult:
        """
        Update detector with current force sample.
        Returns ContactResult indicating trigger state and diagnostics.
        """
        st = self._state
        if st.baseline_mN is None or st.baseline_epoch_s is None:
            return ContactResult(
                triggered=False,
                baseline_mN=None,
                delta_mN=None,
                hits=0,
                reason="NO_BASELINE",
            )

        # Holdoff window after baseline capture
        if self._cfg.holdoff_ms > 0:
            elapsed_ms = (float(epoch_s) - float(st.baseline_epoch_s)) * 1000.0
            if elapsed_ms < self._cfg.holdoff_ms:
                return ContactResult(
                    triggered=False,
                    baseline_mN=st.baseline_mN,
                    delta_mN=int(force_mN) - st.baseline_mN,
                    hits=st.hits,
                    reason="HOLDOFF",
                )

        delta = int(force_mN) - int(st.baseline_mN)

        # Optional sanity clamp
        if self._cfg.max_delta_mN > 0 and delta > self._cfg.max_delta_mN:
            self._state = ContactState(
                baseline_mN=st.baseline_mN,
                baseline_epoch_s=st.baseline_epoch_s,
                hits=0,
            )
            return ContactResult(
                triggered=False,
                baseline_mN=st.baseline_mN,
                delta_mN=delta,
                hits=0,
                reason="SANITY_CLAMP",
            )

        if delta >= self._cfg.rise_threshold_mN:
            hits = st.hits + 1
        else:
            hits = 0

        triggered = hits >= self._cfg.consecutive_hits

        self._state = ContactState(
            baseline_mN=st.baseline_mN,
            baseline_epoch_s=st.baseline_epoch_s,
            hits=hits,
        )

        return ContactResult(
            triggered=triggered,
            baseline_mN=st.baseline_mN,
            delta_mN=delta,
            hits=hits,
            reason="THRESHOLD" if triggered else "",
        )


def build_contact_detector_from_app_config(app_cfg) -> ContactDetector:
    """Adapter from AppConfig -> ContactDetector (V1 only)."""
    cfg = ContactDetectConfig(
        rise_threshold_mN=int(app_cfg.contact.contact_rise_mN),
        holdoff_ms=0,
        consecutive_hits=1,
        max_delta_mN=0,
    )
    return ContactDetector(cfg)


# ============================================================
# V2 — Three-condition contact detection (technical note)
# ============================================================

@dataclass(frozen=True)
class ThreeConditionResult:
    """
    Result from ThreeConditionContactDetector.update().

    triggered is True only when ALL three conditions have been met for
    at least `required_hits` consecutive samples.

    The individual condition flags (force_ok, velocity_ok, position_ok)
    are exposed for logging and diagnostics — they make it easy to see
    in the CSV which condition is blocking detection.
    """
    triggered: bool

    # Individual condition states (True = condition satisfied)
    force_ok: bool       # abs(force - apply_force) <= envelope
    velocity_ok: bool    # abs(velocity) <= stopped_velocity
    position_ok: bool    # position < spring_b_position

    # Raw readings at the time of this sample
    force_mN: int
    velocity_um_s: float
    position_um: int

    # How many consecutive samples all three conditions have held
    consecutive_hits: int


class ThreeConditionContactDetector:
    """
    V2 contact detector: triggers when force, velocity, and position
    conditions all hold simultaneously for a configurable number of
    consecutive loop ticks.

    Matches the detect_contact() logic from the Iris Dynamics technical note:
        triggered = (
            abs(force - apply_force_mN) <= force_envelope_mN  AND
            abs(velocity)               <= stopped_velocity_um_s  AND
            position                    <  spring_b_position_um
        )

    The position condition prevents a spurious trigger while the shaft is
    still moving through the fast-travel zone before reaching the label.

    Args:
        apply_force_mN:         Expected force at contact (from LabelApplicatorConfig).
        force_envelope_mN:      Tolerance band around apply_force_mN.
        stopped_velocity_um_s:  |velocity| threshold for "shaft stopped".
        spring_b_position_um:   Absolute position of Spring B (home_um + extended_offset).
                                Position must be below this for contact to be valid.
        required_hits:          Number of consecutive samples all conditions must hold
                                before triggered=True is returned. Default 1 (no debounce).
                                Increase for noisy force signals.
    """

    def __init__(
        self,
        apply_force_mN: int,
        force_envelope_mN: int,
        stopped_velocity_um_s: int,
        spring_b_position_um: int,
        required_hits: int = 1,
    ):
        if force_envelope_mN <= 0:
            raise ValueError("force_envelope_mN must be > 0")
        if stopped_velocity_um_s <= 0:
            raise ValueError("stopped_velocity_um_s must be > 0")
        if required_hits < 1:
            raise ValueError("required_hits must be >= 1")

        self._apply_force_mN = int(apply_force_mN)
        self._force_envelope_mN = int(force_envelope_mN)
        self._stopped_velocity_um_s = int(stopped_velocity_um_s)
        self._spring_b_position_um = int(spring_b_position_um)
        self._required_hits = int(required_hits)
        self._hits = 0

    def reset(self) -> None:
        """Reset consecutive-hit counter. Call at the start of each apply cycle."""
        self._hits = 0

    def update(
        self,
        force_mN: int,
        velocity_um_s: float,
        position_um: int,
    ) -> ThreeConditionResult:
        """
        Evaluate the three contact conditions against the current motor state.

        Args:
            force_mN:      Current force reading from stream cache (signed, mN).
            velocity_um_s: Derived velocity from OrcaClient.read_stream_cache() (um/s).
            position_um:   Current position from stream cache (um).

        Returns:
            ThreeConditionResult with triggered=True if all three conditions
            have been satisfied for at least required_hits consecutive samples.
        """
        force_ok = abs(int(force_mN) - self._apply_force_mN) <= self._force_envelope_mN
        velocity_ok = abs(float(velocity_um_s)) <= float(self._stopped_velocity_um_s)
        position_ok = int(position_um) < self._spring_b_position_um

        all_met = force_ok and velocity_ok and position_ok

        if all_met:
            self._hits += 1
        else:
            self._hits = 0

        return ThreeConditionResult(
            triggered=self._hits >= self._required_hits,
            force_ok=force_ok,
            velocity_ok=velocity_ok,
            position_ok=position_ok,
            force_mN=int(force_mN),
            velocity_um_s=float(velocity_um_s),
            position_um=int(position_um),
            consecutive_hits=self._hits,
        )


@dataclass(frozen=True)
class EmptyResult:
    """
    Result from EmptyDetector.update().

    triggered is True when the shaft has reached the extended position
    and is stopped there — meaning no label was present during the apply stroke.
    """
    triggered: bool
    position_um: int
    velocity_um_s: float
    consecutive_hits: int


class EmptyDetector:
    """
    V2 empty-label detector: triggers when the shaft reaches the extended
    position (Spring B) and stops there with no contact detected.

    Matches the detect_empty() logic from the Iris Dynamics technical note:
        triggered = (
            position >= spring_b_position_um  AND
            abs(velocity) <= stopped_velocity_um_s
        )

    This is the strict complement of ThreeConditionContactDetector's position
    condition — together they cover all stopped-shaft states.

    Args:
        spring_b_position_um:   Absolute position of Spring B (home_um + extended_offset).
        stopped_velocity_um_s:  |velocity| threshold for "shaft stopped".
        required_hits:          Consecutive samples required before triggering.
    """

    def __init__(
        self,
        spring_b_position_um: int,
        stopped_velocity_um_s: int,
        required_hits: int = 1,
    ):
        if stopped_velocity_um_s <= 0:
            raise ValueError("stopped_velocity_um_s must be > 0")
        if required_hits < 1:
            raise ValueError("required_hits must be >= 1")

        self._spring_b_position_um = int(spring_b_position_um)
        self._stopped_velocity_um_s = int(stopped_velocity_um_s)
        self._required_hits = int(required_hits)
        self._hits = 0

    def reset(self) -> None:
        """Reset consecutive-hit counter. Call at the start of each apply cycle."""
        self._hits = 0

    def update(self, velocity_um_s: float, position_um: int) -> EmptyResult:
        """
        Evaluate whether the shaft has reached the extended position with no label.

        Args:
            velocity_um_s: Derived velocity from OrcaClient.read_stream_cache() (um/s).
            position_um:   Current position from stream cache (um).

        Returns:
            EmptyResult with triggered=True if the shaft is at or past Spring B
            and is stopped, for at least required_hits consecutive samples.
        """
        at_limit = int(position_um) >= self._spring_b_position_um
        stopped = abs(float(velocity_um_s)) <= float(self._stopped_velocity_um_s)

        if at_limit and stopped:
            self._hits += 1
        else:
            self._hits = 0

        return EmptyResult(
            triggered=self._hits >= self._required_hits,
            position_um=int(position_um),
            velocity_um_s=float(velocity_um_s),
            consecutive_hits=self._hits,
        )


def build_v2_contact_detector(
    la_cfg,
    home_um: int,
    required_hits: int = 1,
) -> ThreeConditionContactDetector:
    """
    Factory: build ThreeConditionContactDetector from LabelApplicatorConfig.

    Args:
        la_cfg:        AppConfig.label_applicator (LabelApplicatorConfig).
        home_um:       Absolute home position from autozero_and_home().
        required_hits: Optional debounce; default 1 (no debounce).
    """
    spring_b_position_um = home_um + la_cfg.extended_position_offset_um
    return ThreeConditionContactDetector(
        apply_force_mN=la_cfg.apply_force_mN,
        force_envelope_mN=la_cfg.force_detection_envelope_mN,
        stopped_velocity_um_s=la_cfg.stopped_velocity_um_s,
        spring_b_position_um=spring_b_position_um,
        required_hits=required_hits,
    )


def build_v2_empty_detector(
    la_cfg,
    home_um: int,
    required_hits: int = 1,
) -> EmptyDetector:
    """
    Factory: build EmptyDetector from LabelApplicatorConfig.

    Args:
        la_cfg:        AppConfig.label_applicator (LabelApplicatorConfig).
        home_um:       Absolute home position from autozero_and_home().
        required_hits: Optional debounce; default 1 (no debounce).
    """
    spring_b_position_um = home_um + la_cfg.extended_position_offset_um
    return EmptyDetector(
        spring_b_position_um=spring_b_position_um,
        stopped_velocity_um_s=la_cfg.stopped_velocity_um_s,
        required_hits=required_hits,
    )