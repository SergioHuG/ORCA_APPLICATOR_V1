# src/orca_v1/state_machine.py
"""
state_machine.py — V2 label applicator state machine

Implements the four control states from the Iris Dynamics technical note
and enforces the single-transition-point invariant:

    enter_state() is the ONLY function that changes current_state.
    current_state is NEVER written directly anywhere else.

States:
    AUTO_ZEROING   — initial AutoZero sequence (handled by autozero_and_home()
                     before the state machine becomes active; included for
                     completeness and consistent logging).
    RETURN_HOME    — constant force = 0; Spring A pulls shaft to home.
    APPLY          — constant force = apply_force_mN; shaft accelerates toward label.
    HOLD_CONTACT   — contact detected; constant force held, internal timer running;
                     transitions to RETURN_HOME when timer expires.

Entry actions (executed atomically inside enter_state()):
    RETURN_HOME  → set_constant_force(0)
    APPLY        → set_constant_force(apply_force_mN)
    HOLD_CONTACT → start hold timer (force unchanged — stays at apply_force_mN)
    AUTO_ZEROING → set_constant_force(0)

Runtime flow (V2 haptic-only, after configure_haptic_effects()):
    1. State machine created → initial state = RETURN_HOME, force = 0.
    2. GPIO apply trigger → enter_state(APPLY) → force = apply_force_mN.
    3. Contact detected   → enter_state(HOLD_CONTACT) → timer starts.
    4. Timer expires      → enter_state(RETURN_HOME) → force = 0.
    5. Shaft back at home → cycle complete, back to step 2.

    On empty (no label): APPLY → enter_state(RETURN_HOME) immediately.
    On error: any state → enter_state(RETURN_HOME) as safe fallback.
"""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import Optional

from .orca_client import OrcaClient


class ControlState(Enum):
    """The four V2 control states, mirroring the technical note enum."""
    AUTO_ZEROING  = auto()
    RETURN_HOME   = auto()
    APPLY         = auto()
    HOLD_CONTACT  = auto()


class LabelApplicatorStateMachine:
    """
    V2 label applicator state machine.

    The machine owns its state and the Constant Force register.
    All other code reads self.state and calls enter_state() — nothing
    writes set_constant_force() or mutates self._state directly.

    Args:
        client:                   OrcaClient with an open port in HapticMode.
        apply_force_mN:           Force written on APPLY entry (from la_cfg.apply_force_mN).
        hold_contact_duration_ms: How long to stay in HOLD_CONTACT before returning home.

    Usage:
        sm = LabelApplicatorStateMachine(client, apply_force_mN=20_000,
                                         hold_contact_duration_ms=500)

        # GPIO apply trigger received:
        sm.enter_state(ControlState.APPLY)

        # in control loop:
        if contact_detector.update(...).triggered:
            sm.enter_state(ControlState.HOLD_CONTACT)

        if sm.state == ControlState.HOLD_CONTACT and sm.is_hold_complete():
            sm.enter_state(ControlState.RETURN_HOME)
    """

    def __init__(
        self,
        client: OrcaClient,
        apply_force_mN: int,
        hold_contact_duration_ms: int,
    ):
        if apply_force_mN <= 0:
            raise ValueError(f"apply_force_mN must be > 0, got {apply_force_mN}")
        if hold_contact_duration_ms < 0:
            raise ValueError("hold_contact_duration_ms must be >= 0")

        self._client = client
        self._apply_force_mN = int(apply_force_mN)
        self._hold_contact_duration_ms = int(hold_contact_duration_ms)

        # Initialise in RETURN_HOME and immediately enforce the entry action
        # so the motor's Constant Force register is guaranteed to be 0 from
        # the moment the state machine is created.
        self._state: ControlState = ControlState.RETURN_HOME
        self._hold_start_s: Optional[float] = None
        self._client.set_constant_force(0)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def state(self) -> ControlState:
        """Current control state (read-only). Use enter_state() to change it."""
        return self._state

    @property
    def hold_elapsed_ms(self) -> Optional[float]:
        """
        Elapsed time in the HOLD_CONTACT state (ms).
        None if not currently in HOLD_CONTACT or timer was never started.
        Useful for logging and progress indicators.
        """
        if self._state != ControlState.HOLD_CONTACT or self._hold_start_s is None:
            return None
        return (time.monotonic() - self._hold_start_s) * 1000.0

    def is_hold_complete(self) -> bool:
        """
        True if the HOLD_CONTACT timer has reached hold_contact_duration_ms.
        Always False in any other state.
        """
        elapsed = self.hold_elapsed_ms
        if elapsed is None:
            return False
        return elapsed >= float(self._hold_contact_duration_ms)

    def enter_state(self, new_state: ControlState) -> None:
        """
        Transition to new_state and execute its entry action atomically.

        This is the ONLY function that may change self._state.
        Calling with the current state is a no-op (idempotent).

        Entry actions:
            RETURN_HOME  → set_constant_force(0)
            APPLY        → set_constant_force(apply_force_mN)
            HOLD_CONTACT → start hold timer (force unchanged)
            AUTO_ZEROING → set_constant_force(0)

        Args:
            new_state: The target ControlState.

        Raises:
            OrcaDeviceError: if the SDK call inside the entry action fails.
        """
        if new_state == self._state:
            return  # no-op: already in the requested state

        prev_state = self._state
        self._state = new_state

        if new_state == ControlState.RETURN_HOME:
            # Remove the downward force bias; Spring A pulls shaft to home.
            self._hold_start_s = None
            self._client.set_constant_force(0)

        elif new_state == ControlState.APPLY:
            # Apply the downward force; overcomes Spring A, shaft moves toward label.
            self._hold_start_s = None
            self._client.set_constant_force(self._apply_force_mN)

        elif new_state == ControlState.HOLD_CONTACT:
            # Keep force at apply_force_mN (no register write needed).
            # Start the hold timer so the caller can poll is_hold_complete().
            self._hold_start_s = time.monotonic()
            # Force is already apply_force_mN from the APPLY state —
            # no set_constant_force() call required here.

        elif new_state == ControlState.AUTO_ZEROING:
            # Safe fallback: remove any active force bias.
            self._hold_start_s = None
            self._client.set_constant_force(0)

        # Surface the transition for logging — callers can print this if needed
        print(
            f"[STATE] {prev_state.name} -> {new_state.name}"
            + (f"  (hold_duration={self._hold_contact_duration_ms}ms)"
               if new_state == ControlState.HOLD_CONTACT else "")
        )

    def safe_return(self) -> None:
        """
        Best-effort emergency return: transition to RETURN_HOME unconditionally.

        Use in exception handlers and FAULT paths. Swallows SDK errors so
        it is safe to call even if the motor is in an unknown state.
        """
        try:
            self.enter_state(ControlState.RETURN_HOME)
        except Exception as ex:
            print(f"[STATE] safe_return: enter_state failed ({ex}), "
                  "attempting direct set_constant_force(0)")
            try:
                self._client.set_constant_force(0)
                self._state = ControlState.RETURN_HOME
                self._hold_start_s = None
            except Exception:
                pass


def build_state_machine(
    client: OrcaClient,
    la_cfg,
) -> LabelApplicatorStateMachine:
    """
    Factory: create LabelApplicatorStateMachine from LabelApplicatorConfig.

    Reads apply_force_mN and hold_contact_duration_ms directly from la_cfg,
    writes set_constant_force(0) immediately on construction.

    Args:
        client:  OrcaClient with open port in HapticMode.
        la_cfg:  AppConfig.label_applicator (LabelApplicatorConfig).

    Returns:
        LabelApplicatorStateMachine ready for the control loop.
    """
    return LabelApplicatorStateMachine(
        client=client,
        apply_force_mN=la_cfg.apply_force_mN,
        hold_contact_duration_ms=la_cfg.hold_contact_duration_ms,
    )