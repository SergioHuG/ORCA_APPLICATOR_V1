# src/orca_v1/contact_detect.py
"""
contact_detect.py (V1)

Force-based contact detection using:
- baseline force captured at (or near) haptic entry
- delta = (current_force - baseline)
- trigger when delta >= threshold

V1 additions (safety/robustness, still simple):
- optional holdoff after baseline capture (ignore first N ms)
- optional debounce via N consecutive samples above threshold

This module is pure logic (no SDK calls), easy to unit test.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


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
    Usage pattern (typical):

        det = ContactDetector(ContactDetectConfig(rise_threshold_mN=4000))
        det.capture_baseline(force_mN, epoch_s)

        # each tick while in HAPTIC:
        res = det.update(force_mN, epoch_s)
        if res.triggered:
            ...

    You can also re-use the detector across cycles by calling reset().
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
        """
        Capture baseline force for delta computation.
        Resets hit counter.
        """
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

        # Optional sanity clamp (guards against nonsense spikes causing a false trigger)
        if self._cfg.max_delta_mN > 0 and delta > self._cfg.max_delta_mN:
            # We do not trigger; we also reset hits to be conservative.
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
            hits = 0  # reset when condition not met

        triggered = hits >= self._cfg.consecutive_hits

        # update state
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
    """
    Convenience adapter from AppConfig -> ContactDetector.
    Keeps coupling loose by not importing AppConfig type directly.
    """
    # V1 defaults: no holdoff, no debounce beyond 1 sample, no max_delta clamp
    cfg = ContactDetectConfig(
        rise_threshold_mN=int(app_cfg.contact.contact_rise_mN),
        holdoff_ms=0,
        consecutive_hits=1,
        max_delta_mN=0,
    )
    return ContactDetector(cfg)
