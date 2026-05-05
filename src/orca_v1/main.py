# src/orca_v1/main.py
"""
main.py (V1)

Wires together:
- AppConfig (YAML)
- TriggerQueue
- GPIOInputs (gpiozero + pigpio)
- OrcaClient lifecycle
- Phase sequencing (BOOT/AUTOZERO_HOME/IDLE/APPLY/FAULT)
- Apply sequence implementation (apply_sequence.py)

Sequence goal:
  Boot (or GPIO trigger) -> AutoZero -> Home -> IDLE waiting for APPLY trigger
  APPLY -> HAPTIC bumper -> CONTACT -> RETURN -> back to IDLE

Run:
  python -m orca_v1 --config configs/default.yaml

Notes:
- GPIO callbacks enqueue events only; the main loop consumes them in IDLE.
- APPLY uses stream_cache-first loop inside apply_sequence.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import time
import argparse

from .config import load_config, AppConfig
from .orca_client import OrcaClient, SerialConfig as ClientSerialConfig
from .triggers import TriggerQueue, TriggerType, TriggerEdge, TriggerEvent
from .gpio_inputs import GPIOInputs
from .apply_sequence import autozero_and_home, run_apply_cycle, ApplyCycleResult


class Phase(Enum):
    BOOT = auto()
    AUTOZERO_HOME = auto()
    IDLE = auto()
    APPLY = auto()
    FAULT = auto()


@dataclass
class RuntimeState:
    phase: Phase = Phase.BOOT
    last_event: str = ""


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Orca Applicator V1")
    p.add_argument(
        "--config",
        default="configs/default.yaml",
        help="Path to YAML config (default: configs/default.yaml)",
    )
    return p.parse_args()


def _should_consume_event(ev: TriggerEvent) -> bool:
    """
    V1 policy:
    - We mainly act on PRESSED events.
    - RELEASED is still enqueued (useful for timing/diagnostics) but ignored for actions.
    """
    return ev.edge == TriggerEdge.PRESSED


def _maybe_print_event(ev: TriggerEvent) -> None:
    print(f"[GPIO] t={ev.t_mono_s:.6f}  {ev.trigger.name} {ev.edge.name}  pin={ev.pin}")


def run_controller(cfg: AppConfig) -> int:
    """
    Run the top-level phase controller loop.

    Args:
        cfg: Loaded AppConfig.

    Returns:
        Process-style exit code (0 ok, 1 error).
    """

    # --- Triggers + GPIO ---
    trigger_q = TriggerQueue()
    gpio = GPIOInputs(cfg.gpio, trigger_q)
    gpio.start()

    if cfg.gpio.enabled:
        st = gpio.status()
        print(
            f"[GPIO] enabled backend={st.backend} host={st.pigpio_host} "
            f"autozero_pin={st.autozero_pin} apply_pin={st.apply_pin}"
        )
    else:
        print("[GPIO] disabled")

    # --- Orca client ---
    client_serial = ClientSerialConfig(
        port=cfg.serial.port,
        baudrate=cfg.serial.baudrate,
        interframe_us=cfg.serial.interframe_us,
    )
    client = OrcaClient("OrcaApplicatorV1", serial=client_serial)

    home_um: Optional[int] = None  # persisted across cycles

    try:
        print(
            f"[ORCA] Opening {cfg.serial.port} @ {cfg.serial.baudrate} baud, "
            f"{cfg.serial.interframe_us}us interframe"
        )
        client.open()
        client.clear_errors()
        client.enable_stream()

        # Warm-up stream cache (non-fatal if it doesn't update immediately)
        alive = client.wait_for_stream_alive(timeout_s=2.0)
        if not alive:
            print("[ORCA] Warning: stream_cache did not show updates during warmup window.")

        state = RuntimeState(phase=Phase.BOOT)

        # --- Main loop ---
        idle_poll_dt = 0.01  # 10ms idle poll cadence for responsive trigger handling

        while True:
            if state.phase == Phase.BOOT:
                print(f"[MAIN] phase -> {state.phase.name}")

                if cfg.autozero_home_on_boot:
                    state.phase = Phase.AUTOZERO_HOME
                else:
                    state.phase = Phase.IDLE
                continue

            if state.phase == Phase.AUTOZERO_HOME:
                print(f"[MAIN] phase -> {state.phase.name}")
                home_um = autozero_and_home(client, cfg)
                state.phase = Phase.IDLE
                continue

            if state.phase == Phase.IDLE:
                # Drain events quickly so we don't miss bursts
                events = trigger_q.drain(limit=100)
                action_taken = False

                for ev in events:
                    _maybe_print_event(ev)

                    if not _should_consume_event(ev):
                        continue

                    if ev.trigger == TriggerType.AUTOZERO:
                        print("[IDLE] AutoZero trigger received -> AUTOZERO_HOME")
                        state.phase = Phase.AUTOZERO_HOME
                        action_taken = True
                        break

                    if ev.trigger == TriggerType.APPLY:
                        print("[IDLE] Apply trigger received -> APPLY")
                        state.phase = Phase.APPLY
                        action_taken = True
                        break

                if action_taken:
                    continue

                # Stay idle
                time.sleep(idle_poll_dt)
                continue

            if state.phase == Phase.APPLY:
                print(f"[MAIN] phase -> {state.phase.name}")

                # Ensure we have a valid HOME reference before applying
                if home_um is None:
                    print("[APPLY] No HOME known yet -> running AutoZero/Home first")
                    home_um = autozero_and_home(client, cfg)

                res: ApplyCycleResult = run_apply_cycle(client, cfg, home_um=home_um)
                print(
                    f"[APPLY] ok={res.ok} reason={res.reason} "
                    f"baseline={res.baseline_mN} delta={res.contact_delta_mN} "
                    f"peak_force={res.peak_force_mN} elapsed={res.elapsed_s:.3f}s "
                    f"csv={res.csv_path}"
                )

                # If apply failed in a way that might invalidate HOME, you can decide to clear it.
                # For V1 we keep HOME unless you want stricter behavior.
                state.phase = Phase.IDLE
                continue

            if state.phase == Phase.FAULT:
                print("[MAIN] FAULT: safe stop and exit")
                client.safe_stop()
                return 1

    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted. Exiting.")
        return 0
    except Exception as ex:
        print(f"\n[MAIN] ERROR: {ex}")
        try:
            client.safe_stop()
        except Exception:
            pass
        return 1
    finally:
        try:
            gpio.close()
        except Exception:
            pass
        try:
            client.close()
        except Exception:
            pass


def main() -> int:
    """CLI entrypoint: parse args, load config, and run controller."""

    args = _parse_args()
    cfg = load_config(args.config)
    return run_controller(cfg)


if __name__ == "__main__":
    raise SystemExit(main())


