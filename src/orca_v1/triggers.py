# src/orca_v1/triggers.py
"""
triggers.py (V1)

A tiny, thread-safe trigger event layer.

Design goals:
- GPIO callbacks should be *fast*: enqueue a small event and return.
- Main loop should poll non-blocking and decide what to do (IDLE -> APPLY, etc.).
- Optional software lockout (per trigger) to avoid bounce/noise storms,
  especially for APPLY where bounce_time_s may be None.

This module is pure Python/stdlib and safe to unit test.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Dict
import time
import queue


class TriggerType(Enum):
    AUTOZERO = auto()
    APPLY = auto()


class TriggerEdge(Enum):
    PRESSED = auto()
    RELEASED = auto()


@dataclass(frozen=True)
class TriggerEvent:
    t_mono_s: float           # time.monotonic() for relative timing
    trigger: TriggerType
    edge: TriggerEdge
    pin: Optional[int] = None


class TriggerQueue:
    """
    Thread-safe queue for trigger events.

    - put_nowait() is safe from gpiozero callback threads.
    - poll() is non-blocking for the main loop.
    """

    def __init__(self, maxsize: int = 256):
        self._q: "queue.Queue[TriggerEvent]" = queue.Queue(maxsize=maxsize)

    def put_nowait(self, ev: TriggerEvent) -> None:
        try:
            self._q.put_nowait(ev)
        except queue.Full:
            # If flooded, drop newest to preserve callback responsiveness.
            # (You can change this policy later.)
            pass

    def poll(self) -> Optional[TriggerEvent]:
        """Non-blocking: returns one event if available, else None."""
        try:
            return self._q.get_nowait()
        except queue.Empty:
            return None

    def drain(self, limit: int = 1000) -> list[TriggerEvent]:
        """Drain up to `limit` events quickly (useful for IDLE processing)."""
        out: list[TriggerEvent] = []
        for _ in range(limit):
            ev = self.poll()
            if ev is None:
                break
            out.append(ev)
        return out

    def size_approx(self) -> int:
        """Approximate queue size (not reliable under concurrency)."""
        try:
            return self._q.qsize()
        except Exception:
            return 0


class TriggerLockout:
    """
    Software lockout per TriggerType.

    Use this to prevent duplicate triggers within a window, even when
    gpiozero bounce_time_s is None (fast apply input).

    Typical usage in gpio_inputs.py callback:
        if lockout.allow(TriggerType.APPLY, now_mono):
            queue.put_nowait(...)
    """

    def __init__(self):
        self._last_fire_mono: Dict[TriggerType, float] = {}

    def allow(self, trig: TriggerType, now_mono_s: Optional[float] = None, lockout_ms: int = 0) -> bool:
        """
        Returns True if allowed, False if within lockout window.
        Updates last-fire time when allowed.
        """
        if lockout_ms <= 0:
            # No lockout requested
            self._last_fire_mono[trig] = now_mono_s if now_mono_s is not None else time.monotonic()
            return True

        now = now_mono_s if now_mono_s is not None else time.monotonic()
        last = self._last_fire_mono.get(trig)
        if last is None:
            self._last_fire_mono[trig] = now
            return True

        dt_ms = (now - last) * 1000.0
        if dt_ms >= float(lockout_ms):
            self._last_fire_mono[trig] = now
            return True

        return False
