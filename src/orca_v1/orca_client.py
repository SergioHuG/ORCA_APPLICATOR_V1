# src/orca_v1/orca_client.py
"""
orca_client.py (V1)

Thin wrapper around pyorcasdk.Actuator for:
- open_serial_port(/dev/orca0) over Modbus RTU
- clear_errors + enable_stream
- high-speed loop support: run() + stream_cache reads via get_stream_data()
- safe mode changes + motion triggers (kinematic + haptics)
- basic health helpers (stream alive / stale)

Design goals (V1):
- Keep behavior close to your working test script.
- Prefer stream_cache reads (non-blocking) over blocking Modbus reads.
- Safety-first: explicit error raising, safe_stop hooks.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple
import time

from pyorcasdk import Actuator, MotorMode


# -----------------------------
# Exceptions / error handling
# -----------------------------
class OrcaError(RuntimeError):
    """Base exception for Orca client errors."""


class OrcaCommError(OrcaError):
    """Serial/transport or communication related errors."""


class OrcaDeviceError(OrcaError):
    """Device returned an error, fault, or unexpected state."""


def _err_to_str(err) -> str:
    """pyorcasdk returns either None/falsey or an object with what()."""
    try:
        return err.what()
    except Exception:
        return str(err)


def _raise_if_err(err, label: str) -> None:
    if err:
        raise OrcaDeviceError(f"{label}: {_err_to_str(err)}")


# -----------------------------
# Typed stream snapshot
# -----------------------------
@dataclass(frozen=True)
class StreamSnapshot:
    """
    A typed snapshot of the SDK stream cache.

    Fields from SDK stream data: position, force, power, voltage, temperature, errors.
    velocity_um_s is derived from position deltas inside OrcaClient.read_stream_cache()
    because pyorcasdk does not expose velocity in the stream cache directly.
    Defaults to 0.0 so V1 code that ignores velocity is unaffected.
    """
    t_epoch_s: float
    position_um: int
    force_mN: int
    power: int
    voltage: int
    temperature: int
    errors: int
    velocity_um_s: float = 0.0


@dataclass(frozen=True)
class SerialConfig:
    port: str = "/dev/orca0"
    baudrate: int = 1_000_000
    interframe_us: int = 80


# -----------------------------
# Client
# -----------------------------
class OrcaClient:
    """
    Thin wrapper around pyorcasdk.Actuator.

    Typical usage (V1):
        client = OrcaClient("OrcaApplicator", SerialConfig(...))
        client.open()
        client.clear_errors()
        client.enable_stream()

        # in loop:
        client.run()
        snap = client.read_stream_cache()
    """

    def __init__(self, name: str, serial: Optional[SerialConfig] = None):
        self._name = name
        self._serial = serial or SerialConfig()
        self._motor = Actuator(name)

        # Track last stream tuple to detect staleness
        self._last_stream_tuple: Optional[Tuple[int, int, int, int, int, int]] = None
        self._is_open = False

        # Velocity tracking: derived from position deltas since SDK does not expose velocity.
        # Reset between cycles via reset_velocity_tracker().
        self._vel_last_pos_um: Optional[int] = None
        self._vel_last_t: Optional[float] = None

    @property
    def motor(self) -> Actuator:
        """Direct access (escape hatch) for V1 only."""
        return self._motor

    @property
    def serial(self) -> SerialConfig:
        return self._serial

    # ---- lifecycle ----
    def open(self) -> None:
        """Open serial port."""
        err = self._motor.open_serial_port(self._serial.port, self._serial.baudrate, self._serial.interframe_us)
        if err:
            raise OrcaCommError(f"open_serial_port failed: {_err_to_str(err)}")
        self._is_open = True

    def close(self) -> None:
        """
        pyorcasdk may or may not expose a close method depending on version.
        Keep as a placeholder; safe to call even if not implemented.
        """
        self._is_open = False
        # If SDK later provides close, add it here.

    def clear_errors(self) -> None:
        """Clear device errors (best-effort)."""
        # clear_errors() typically doesn't return an error object
        try:
            self._motor.clear_errors()
        except Exception as ex:
            raise OrcaDeviceError(f"clear_errors failed: {ex}") from ex

    def enable_stream(self) -> None:
        """Enable device streaming so stream_cache updates when run() is called."""
        try:
            self._motor.enable_stream()
        except Exception as ex:
            raise OrcaDeviceError(f"enable_stream failed: {ex}") from ex

    # ---- core high-speed comm step ----
    def run(self) -> None:
        """
        Keep comms alive and refresh SDK stream_cache.
        Must be called in your high-speed loop.
        """
        try:
            self._motor.run()
        except Exception as ex:
            raise OrcaCommError(f"motor.run() failed: {ex}") from ex

    # ---- stream_cache reads (non-blocking) ----
    def read_stream_cache(self) -> StreamSnapshot:
        """
        Read cached stream values (non-blocking; uses SDK's last received frame).
        Assumes run() is being called regularly.

        velocity_um_s is derived from the position delta between consecutive calls.
        Call reset_velocity_tracker() at the start of each apply cycle to prevent
        a stale delta from the previous cycle polluting the first sample.
        """
        sd = self._motor.get_stream_data()
        now = time.time()
        pos_um = int(sd.position)

        # Derive velocity from position delta
        vel_um_s = 0.0
        if self._vel_last_pos_um is not None and self._vel_last_t is not None:
            dt = max(1e-6, now - self._vel_last_t)
            vel_um_s = (pos_um - self._vel_last_pos_um) / dt
        self._vel_last_pos_um = pos_um
        self._vel_last_t = now

        return StreamSnapshot(
            t_epoch_s=now,
            position_um=pos_um,
            force_mN=int(sd.force),
            power=int(sd.power),
            voltage=int(sd.voltage),
            temperature=int(sd.temperature),
            errors=int(sd.errors),
            velocity_um_s=vel_um_s,
        )

    def stream_tuple(self) -> Tuple[int, int, int, int, int, int]:
        """Convenience tuple for change detection."""
        sd = self._motor.get_stream_data()
        return (int(sd.position), int(sd.force), int(sd.power), int(sd.voltage), int(sd.temperature), int(sd.errors))

    def wait_for_stream_alive(self, timeout_s: float = 2.0, sleep_s: float = 0.002) -> bool:
        """
        Warm-up helper: call run() until stream_cache changes at least once.
        Returns True if we observed updates, False otherwise.
        Not fatal by itself (you can choose how to react).
        """
        t0 = time.time()
        last = None
        while (time.time() - t0) < timeout_s:
            self.run()
            cur = self.stream_tuple()
            if last is None:
                last = cur
            else:
                if cur != last:
                    self._last_stream_tuple = cur
                    return True
            time.sleep(sleep_s)
        return False

    def is_stream_stale(self, snap: StreamSnapshot) -> bool:
        """
        Simple stale heuristic:
        - If we've never seen a valid stream tuple and fields are all zero, treat as stale.
        - If tuple doesn't change over time, the caller can decide to treat as stale via wait_for_stream_alive.
        """
        tup = (snap.position_um, snap.force_mN, snap.power, snap.voltage, snap.temperature, snap.errors)
        if self._last_stream_tuple is None:
            # Common "not ready" symptom in your script
            if tup == (0, 0, 0, 0, 0, 0):
                return True
        return False

    # ---- mode helpers ----
    def get_mode(self) -> MotorMode:
        m = self._motor.get_mode()
        if getattr(m, "error", None):
            raise OrcaDeviceError(f"get_mode error: {_err_to_str(m.error)}")
        return m.value

    def set_mode(self, mode: MotorMode) -> None:
        _raise_if_err(self._motor.set_mode(mode), f"set_mode({mode})")

    def wait_until_not_mode(self, mode: MotorMode, timeout_s: float, poll_s: float = 0.05) -> None:
        """
        Wait until device mode changes away from `mode`. Used for AutoZero completion.
        """
        t0 = time.time()
        while (time.time() - t0) < timeout_s:
            m = self._motor.get_mode()
            if (not m.error) and (m.value != mode):
                return
            time.sleep(poll_s)
        raise OrcaDeviceError(f"Timeout: still in {mode}")

    # ---- kinematic helpers ----
    def set_kinematic_motion(
        self,
        motion_id: int,
        target_um: int,
        time_ms: int,
        delay_ms: int,
        motion_type: int,
        auto_next: int = 0,
        next_id: int = 0,
    ) -> None:
        err = self._motor.set_kinematic_motion(
            motion_id,
            int(target_um),
            int(time_ms),
            int(delay_ms),
            int(motion_type),
            int(auto_next),
            int(next_id),
        )
        _raise_if_err(err, f"set_kinematic_motion(id={motion_id})")

    def trigger_kinematic_motion(self, motion_id: int) -> None:
        _raise_if_err(self._motor.trigger_kinematic_motion(int(motion_id)), f"trigger_kinematic_motion({motion_id})")

    def get_position_um_blocking(self) -> int:
        """
        Blocking read (Modbus request). Avoid in high-speed loop; useful for bootstrapping.
        """
        p = self._motor.get_position_um()
        if p.error:
            raise OrcaCommError(f"get_position_um error: {_err_to_str(p.error)}")
        return int(p.value)

    def reset_velocity_tracker(self) -> None:
        """
        Reset the internal position-delta velocity tracker.

        Call this at the start of each apply cycle so the first velocity sample
        is not corrupted by the position change from the previous cycle end.
        """
        self._vel_last_pos_um = None
        self._vel_last_t = None

    # ---- haptic force control ----
    def set_constant_force(self, force_mN: int) -> None:
        """
        Write a constant force bias to the motor (mN).

        This is the sole runtime state trigger for the technical-note label applicator:
          - Write apply_force_mN  -> overcomes Spring A, shaft accelerates toward label.
          - Write 0               -> Spring A restores shaft to home position.

        Sign convention (shaft pointing down, gravity assists extension):
          Positive  = downward (extension direction).
          Negative  = upward   (retraction direction).

        Must be called while motor is in HapticMode. Safe to call every loop tick
        with the same value; the motor discards redundant writes efficiently.
        """
        _raise_if_err(
            self._motor.set_constant_force(int(force_mN)),
            f"set_constant_force({force_mN})",
        )

    # ---- safety helpers ----
    def safe_stop(self) -> None:
        """
        Best-effort safe stop hook for V1.
        We keep it minimal until you define your exact stop behavior.
        """
        try:
            # A conservative default: switch to SleepMode if available
            self._motor.set_mode(MotorMode.SleepMode)
        except Exception:
            # Don't raise during emergency stop attempts
            pass