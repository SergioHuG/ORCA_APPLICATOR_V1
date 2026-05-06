# src/orca_v1/main.py
"""
main.py

Wires together:
- AppConfig (YAML)
- TriggerQueue
- GPIOInputs (gpiozero + pigpio)
- OrcaClient lifecycle
- Phase sequencing (BOOT/AUTOZERO_HOME/IDLE/APPLY/FAULT)
- Apply sequence implementation (apply_sequence.py)

V1 mode (cfg.label_applicator is None):
  Boot -> AutoZero -> Home -> IDLE
  APPLY -> kinematic stroke -> haptic soft bumper -> contact delta -> kinematic RETURN -> IDLE

V2 mode (cfg.label_applicator is configured):
  Boot -> AutoZero -> Home -> configure_haptic_effects (HapticMode, stays for session)
       -> IDLE
  APPLY -> set_constant_force (via state machine) -> three-condition contact
        -> HOLD_CONTACT -> set_constant_force(0) -> Spring A return -> IDLE

Run:
  python -m orca_v1 --config configs/default.yaml

Notes:
- GPIO callbacks enqueue events only; the main loop consumes them in IDLE.
- V2 state machine (sm) is created once per AutoZero and reused across cycles.
- V2: motor stays in HapticMode for the entire session after AUTOZERO_HOME.
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
from .haptics import configure_haptic_effects
from .state_machine import LabelApplicatorStateMachine, build_state_machine
from .apply_sequence import (
    autozero_and_home,
    run_apply_cycle,
    ApplyCycleResult,
    run_v2_apply_cycle,
    ApplyV2CycleResult,
)


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
    p = argparse.ArgumentParser(description="Orca Applicator V1/V2")
    p.add_argument(
        "--config",
        default="configs/default.yaml",
        help="Path to YAML config (default: configs/default.yaml)",
    )
    return p.parse_args()


def _should_consume_event(ev: TriggerEvent) -> bool:
    """
    Act on PRESSED events only.
    RELEASED is still enqueued (useful for timing/diagnostics) but ignored for actions.
    """
    return ev.edge == TriggerEdge.PRESSED


def _maybe_print_event(ev: TriggerEvent) -> None:
    print(f"[GPIO] t={ev.t_mono_s:.6f}  {ev.trigger.name} {ev.edge.name}  pin={ev.pin}")


def run_controller(cfg: AppConfig) -> int:
    """
    Run the top-level phase controller loop.

    Selects V1 or V2 apply cycle based on whether cfg.label_applicator is set:
      - None  -> V1 (kinematic-then-haptic, existing behaviour)
      - set   -> V2 (haptic-only, technical-note architecture)

    Args:
        cfg: Loaded AppConfig.

    Returns:
        Process-style exit code (0 ok, 1 error).
    """
    is_v2 = cfg.label_applicator is not None

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

    # Persisted across cycles
    home_um: Optional[int] = None
    sm: Optional[LabelApplicatorStateMachine] = None  # V2 only

    print(f"[MAIN] mode={'V2 (haptic-only)' if is_v2 else 'V1 (kinematic-then-haptic)'}")

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

            # ---- BOOT ----
            if state.phase == Phase.BOOT:
                print(f"[MAIN] phase -> {state.phase.name}")
                state.phase = Phase.AUTOZERO_HOME if cfg.autozero_home_on_boot else Phase.IDLE
                continue

            # ---- AUTOZERO_HOME ----
            if state.phase == Phase.AUTOZERO_HOME:
                print(f"[MAIN] phase -> {state.phase.name}")

                home_um = autozero_and_home(client, cfg)

                if is_v2:
                    # Configure all three springs + damper and transition to HapticMode.
                    # Motor stays in HapticMode for the rest of the session — this is
                    # the only place set_mode(HapticMode) is called.
                    la = cfg.label_applicator
                    configure_haptic_effects(client, la, home_um)
                    sm = build_state_machine(client, la)
                    print(
                        f"[MAIN] V2 haptic effects configured. "
                        f"home_um={home_um}um  "
                        f"apply_force={la.apply_force_mN}mN  "
                        f"spring_b={home_um + la.extended_position_offset_um}um  "
                        f"spring_c={home_um + la.spring_c_position_offset_um}um"
                    )

                state.phase = Phase.IDLE
                continue

            # ---- IDLE ----
            if state.phase == Phase.IDLE:
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

                time.sleep(idle_poll_dt)
                continue

            # ---- APPLY ----
            if state.phase == Phase.APPLY:
                print(f"[MAIN] phase -> {state.phase.name}")

                # Ensure HOME is known before applying
                if home_um is None:
                    print("[APPLY] No HOME known yet -> running AutoZero/Home first")
                    home_um = autozero_and_home(client, cfg)

                    if is_v2:
                        la = cfg.label_applicator
                        configure_haptic_effects(client, la, home_um)
                        sm = build_state_machine(client, la)

                if is_v2:
                    # Guard: state machine must exist (it should after AUTOZERO_HOME)
                    if sm is None:
                        print("[APPLY] V2: state machine missing -> configuring now")
                        la = cfg.label_applicator
                        configure_haptic_effects(client, la, home_um)
                        sm = build_state_machine(client, la)

                    v2res: ApplyV2CycleResult = run_v2_apply_cycle(
                        client, cfg, home_um=home_um, sm=sm
                    )
                    print(
                        f"[APPLY V2] ok={v2res.ok} reason={v2res.reason} "
                        f"contact={v2res.contact_detected} "
                        f"apply_force={v2res.apply_force_mN}mN "
                        f"elapsed={v2res.elapsed_s:.3f}s "
                        f"csv={v2res.csv_path}"
                    )

                else:
                    res: ApplyCycleResult = run_apply_cycle(client, cfg, home_um=home_um)
                    print(
                        f"[APPLY V1] ok={res.ok} reason={res.reason} "
                        f"baseline={res.baseline_mN} delta={res.contact_delta_mN} "
                        f"peak_force={res.peak_force_mN} elapsed={res.elapsed_s:.3f}s "
                        f"csv={res.csv_path}"
                    )

                state.phase = Phase.IDLE
                continue

            # ---- FAULT ----
            if state.phase == Phase.FAULT:
                print("[MAIN] FAULT: safe stop and exit")
                if sm is not None:
                    sm.safe_return()
                client.safe_stop()
                return 1

    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted. Exiting.")
        return 0

    except Exception as ex:
        print(f"\n[MAIN] ERROR: {ex}")
        try:
            if sm is not None:
                sm.safe_return()
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