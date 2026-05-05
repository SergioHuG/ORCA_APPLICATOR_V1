# Orca Applicator V1

## Project Overview
Orca Applicator V1 is a Raspberry Pi Python control app for an Iris Dynamics Orca linear motor applicator over Modbus RTU (`/dev/orca0`). It runs a simple phase flow that performs AutoZero/HOME, waits for a GPIO trigger, executes a kinematic APPLY move, enters a HAPTIC bumper, detects CONTACT by force delta, and returns HOME.

Phase flow (high level):
- BOOT: optionally AutoZero + HOME on boot
- AUTOZERO_HOME: AutoZero, compute HOME, program motions, move HOME
- IDLE: wait for GPIO triggers
- APPLY: APPLY motion, HAPTIC bumper, CONTACT trigger, RETURN HOME
- FAULT: reserved for safe stop on errors

Streaming policy:
- High-speed apply loop uses stream cache reads (`enable_stream()` + `run()` + cache reads).
- Blocking Modbus reads are only used for setup or fallbacks.

## Safety Disclaimer
This is an industrial actuator. Validate on safe fixtures, keep clear of moving parts, and use conservative parameters during setup. Perform a single-cycle sanity check before repeated cycles.

## Requirements
- Raspberry Pi OS
- `pigpiod` running (GPIO backend)
- Iris Dynamics Orca actuator connected via Modbus RTU (`/dev/orca0`)
- Wiring uses BCM GPIO numbers (see `configs/default.yaml`)
- Python packages (install in a venv): `gpiozero==2.0.1`, `pigpio==1.78`, `pyorcasdk==1.1.0`, `PyYAML`

## Setup
Create and activate a venv, then install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Start pigpio daemon:

```bash
sudo systemctl enable --now pigpiod
```

## Run
From the repo root:

```bash
PYTHONPATH=src python -m orca_v1 --config configs/default.yaml
```

## Configuration
All runtime configuration is in `configs/default.yaml` and mapped by `src/orca_v1/config.py`.

High-level keys:
- `serial`: Modbus RTU port and timing (`port`, `baudrate`, `interframe_us`).
- `loop`: high-speed control rate (`control_hz`).
- `motion`: HOME offset, motion IDs, timing, travel, and haptics entry distance.
- `contact`: force delta threshold (`contact_rise_mN`).
- `haptics`: spring and damper gains for the HAPTIC bumper.
- `timeouts`: AutoZero, HOME, RETURN timeouts.
- `tolerances`: HOME position tolerance.
- `logging`: log directory and CSV prefix.
- `gpio`: enable flag, backend, host, and per-button settings (`pin`, `pull_up`, `bounce_time_s`, `lockout_ms`).
- `behavior`: sequencing flags (`autozero_home_on_boot`).

If a value is unclear, confirm it in `src/orca_v1/config.py` before changing the YAML.

## GPIO Triggers
- AutoZero button and Apply button are configured under `gpio.autozero` and `gpio.apply`.
- PRESSED events drive actions; RELEASED events are logged but not used for state changes.
- `bounce_time_s` controls gpiozero debounce. Set to `null` for fastest response.
- `lockout_ms` is a software lockout applied in callbacks to prevent repeated triggers.
- Callbacks only enqueue events; they do not call Modbus or the Orca SDK.

## Logging
Each apply cycle writes one CSV to `logs/` (prefix from `logging.csv_prefix`).

Columns:
- `epoch_s`, `phase`, `elapsed_s`, `position_um`, `velocity_um_s`
- `force_raw_mN`, `baseline_mN`, `delta_mN`, `event`

Common events to look for:
- `ENTER_HAPTIC`
- `CONTACT_TRIGGER`
- `HOME_REACHED`

## Troubleshooting
- `No module named orca_v1`: run with `PYTHONPATH=src` or install the package editable.
- `No module named yaml`: install `PyYAML` (via `requirements.txt`).
- GPIO not responding: ensure `pigpiod` is running and `gpio.backend` is `pigpio`.

## Validation Workflow (Recommended)
- Run a single apply cycle to sanity-check motion and logging.
- Run 10-20 cycles for repeatability checks.
- Inspect CSVs for `ENTER_HAPTIC`, `CONTACT_TRIGGER`, and `HOME_REACHED` events.

## Current Limitations (V1)
- Validation stage: fault supervision is minimal and `FAULT` is only used on exceptions.
- Stream-cache health is best-effort; the loop logs `STREAM_NOT_READY` and continues.
- Conservative timing and safety checks remain in place and may be tuned later.
