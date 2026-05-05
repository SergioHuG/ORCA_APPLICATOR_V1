# src/orca_v1/gpio_inputs.py
"""
gpio_inputs.py (V1)

GPIO input layer using gpiozero + pigpio (PiGPIOFactory).

Based on your proven test script approach:
- Force pigpio pin factory (works well with venv)
- GPIO 5  -> AutoZero trigger (debounced)
- GPIO 6  -> Apply trigger (no debounce, fastest), but with software lockout

Design rules:
- GPIO callbacks must be FAST and must NOT call Orca/Modbus.
- Callbacks only enqueue TriggerEvents into TriggerQueue.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from gpiozero import Button, Device
from gpiozero.pins.pigpio import PiGPIOFactory

from .config import GPIOConfig, GPIOButtonConfig
from .triggers import TriggerQueue, TriggerEvent, TriggerType, TriggerEdge, TriggerLockout

import time


@dataclass(frozen=True)
class GPIOStatus:
    enabled: bool
    backend: str
    pigpio_host: str
    autozero_pin: Optional[int]
    apply_pin: Optional[int]


class GPIOInputs:
    """
    Owns gpiozero Button objects and wires callbacks to a TriggerQueue.

    Typical usage:
        tq = TriggerQueue()
        gpio = GPIOInputs(cfg.gpio, tq)
        gpio.start()

        # main loop polls tq.poll()

        gpio.close()   # best-effort
    """

    def __init__(self, cfg: GPIOConfig, trigger_queue: TriggerQueue):
        self._cfg = cfg
        self._q = trigger_queue

        self._lockout = TriggerLockout()

        self._autozero_btn: Optional[Button] = None
        self._apply_btn: Optional[Button] = None

        self._started = False

    def status(self) -> GPIOStatus:
        return GPIOStatus(
            enabled=self._cfg.enabled,
            backend=self._cfg.backend,
            pigpio_host=self._cfg.pigpio_host,
            autozero_pin=self._cfg.autozero.pin,
            apply_pin=self._cfg.apply.pin,
        )

    def start(self) -> None:
        """
        Initialize GPIO backend and create buttons.
        Safe to call once.
        """
        if self._started:
            return
        self._started = True

        if not self._cfg.enabled:
            return

        if self._cfg.backend != "pigpio":
            raise ValueError(f"Unsupported GPIO backend for V1: {self._cfg.backend}")

        # Force pigpio pin factory (important for venv + low-latency callbacks)
        Device.pin_factory = PiGPIOFactory(host=self._cfg.pigpio_host)

        # Create buttons if pins are configured
        if self._cfg.autozero.pin is not None:
            self._autozero_btn = self._make_button(
                btn_cfg=self._cfg.autozero,
                trig=TriggerType.AUTOZERO,
            )

        if self._cfg.apply.pin is not None:
            self._apply_btn = self._make_button(
                btn_cfg=self._cfg.apply,
                trig=TriggerType.APPLY,
            )

    def close(self) -> None:
        """
        Best-effort cleanup.
        gpiozero Buttons have a close() method to release resources.
        """
        try:
            if self._autozero_btn is not None:
                self._autozero_btn.close()
        except Exception:
            pass
        finally:
            self._autozero_btn = None

        try:
            if self._apply_btn is not None:
                self._apply_btn.close()
        except Exception:
            pass
        finally:
            self._apply_btn = None

    # -----------------------------
    # Internal helpers
    # -----------------------------
    def _make_button(self, btn_cfg: GPIOButtonConfig, trig: TriggerType) -> Button:
        """
        Create a gpiozero.Button and attach press/release callbacks.
        """
        bounce = btn_cfg.bounce_time_s  # None => fastest possible
        pin = int(btn_cfg.pin) if btn_cfg.pin is not None else None
        if pin is None:
            raise ValueError("Button pin must not be None here")

        b = Button(
            pin,
            pull_up=bool(btn_cfg.pull_up),
            bounce_time=bounce,
        )

        # Attach callbacks
        b.when_pressed = self._make_callback(trig, TriggerEdge.PRESSED, btn_cfg.lockout_ms, pin)
        b.when_released = self._make_callback(trig, TriggerEdge.RELEASED, btn_cfg.lockout_ms, pin)

        return b

    def _make_callback(self, trig: TriggerType, edge: TriggerEdge, lockout_ms: int, pin: int):
        """
        Returns a closure that enqueues TriggerEvent if allowed by lockout.
        """
        def _cb():
            now = time.monotonic()

            # Lockout typically should apply to PRESSED (not RELEASED),
            # but applying it to both reduces event spam.
            allowed = self._lockout.allow(trig, now_mono_s=now, lockout_ms=int(lockout_ms))
            if not allowed:
                return

            self._q.put_nowait(
                TriggerEvent(
                    t_mono_s=now,
                    trigger=trig,
                    edge=edge,
                    pin=pin,
                )
            )
        return _cb
