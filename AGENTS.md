# AGENTS.md — Codex Instructions (Orca Applicator V1)

This repository is a Raspberry Pi Python control application for an Iris Dynamics Orca linear motor applicator using `pyorcasdk` over Modbus RTU (`/dev/orca0`). The project is in V1 validation phase: architecture is established; we are documenting and commenting the current behavior, not refactoring or adding features.

## Primary goals for Codex
1) Write a clear, accurate `README.md` that documents:
   - What the app does (phases + flow)
   - How to run it in a venv
   - Required services (pigpiod)
   - How GPIO triggers work
   - Configuration via `configs/default.yaml`
   - Logging output (CSV) and where to find it
   - Known limitations (V1)
2) Add missing comments/docstrings to code files:
   - Module-level docstrings if missing
   - Function docstrings when unclear
   - Inline comments only where needed (avoid noise)
   - Keep comments accurate, actionable, and consistent with current behavior

## Non-goals (important)
- Do NOT change runtime behavior.
- Do NOT refactor logic, rename public functions, or reorganize files.
- Do NOT “optimize” loops, timings, or hardware calls.
- Do NOT introduce new dependencies.
- Do NOT remove logging or safety checks.
- Do NOT change configuration schema unless explicitly asked.
- Do NOT add new files except README content (and comments/docstrings in existing files).

If documentation reveals a bug or inconsistency, mention it in a `Known Issues` section in README, but do not change the code unless asked.

---

## Repository structure (authoritative)
- `src/orca_v1/`
  - `__main__.py` — package entrypoint; calls `orca_v1.main.main()`
  - `main.py` — orchestrates phases + GPIO triggers; calls `apply_sequence`
  - `config.py` — dataclass config schema + YAML loader
  - `orca_client.py` — wrapper/abstraction around `pyorcasdk` for serial + streaming + helpers
  - `apply_sequence.py` — implements APPLY cycle (kinematic -> haptics -> contact -> return) + AutoZero/Home helper
  - `gpio_inputs.py` — gpiozero + pigpio Button setup; callbacks enqueue events
  - `triggers.py` — TriggerEvent types + TriggerQueue + lockout logic
  - `haptics.py` — soft bumper (spring + damper) configuration + keep-alive
  - `contact_detect.py` — baseline + delta force detection logic
  - `logging_csv.py` — CSV telemetry logger
- `configs/default.yaml` — runtime configuration (serial, loop rate, motion params, thresholds, GPIO config)
- `logs/` — generated CSV logs per apply cycle
- `README.md` — currently empty; must be written accurately

---

## Current runtime behavior (must match README + comments)
### Phase flow
The app follows this high-level state machine:

- `BOOT`
  - if `behavior.autozero_home_on_boot: true` → go to `AUTOZERO_HOME`
  - else → go to `IDLE`

- `AUTOZERO_HOME`
  - runs `autozero_and_home()`:
    - enters AutoZero mode
    - waits until AutoZero completes
    - reads position, computes HOME = pos_after + home_offset
    - programs kinematic motions HOME/RETURN/APPLY
    - switches to KinematicMode and triggers HOME
  - transitions to `IDLE`

- `IDLE`
  - drains GPIO-trigger events from `TriggerQueue`
  - on `AUTOZERO PRESSED` → `AUTOZERO_HOME`
  - on `APPLY PRESSED` → `APPLY`
  - ignores RELEASED for actions (still logged to console)

- `APPLY`
  - if HOME unknown, runs `autozero_and_home()` first
  - runs `run_apply_cycle()`:
    - triggers APPLY kinematic motion
    - enters HAPTIC bumper at `haptic_entry_um`
    - captures baseline force at haptic entry
    - monitors delta force; on threshold triggers return
    - triggers RETURN motion and waits for HOME reached
    - writes one CSV log per cycle
  - returns to `IDLE`

- `FAULT` (reserved)
  - safe stop and exit (only used on errors)

### Streaming policy
- High-speed portion uses non-blocking reads from stream cache (`enable_stream()` + `run()` + cache reads).
- Avoid blocking Modbus reads in the high-speed apply loop.
- Blocking reads are allowed for setup steps (AutoZero completion, occasional fallback).

### GPIO policy
- GPIO uses `gpiozero` with `pigpio` pin factory.
- Callbacks must enqueue events only; they must not call Orca/Modbus.
- Apply can run with `bounce_time_s: null` for fastest response, but software lockout applies.

### Logging policy
- One CSV file per apply cycle in `logs/`
- Columns: `epoch_s, phase, elapsed_s, position_um, velocity_um_s, force_raw_mN, baseline_mN, delta_mN, event`
- README must explain how to inspect these logs.

---

## Documentation tasks (step-by-step for Codex)

### Task A — Write README.md (high priority)
Create a complete README with these sections:

1) Project Overview
   - What the Orca Applicator V1 does in plain language
2) Safety Disclaimer
   - Industrial actuator; use care; validate on safe fixtures first
3) Requirements
   - Raspberry Pi OS, pigpiod, wiring notes
4) Setup
   - Create venv and install dependencies:
     - `gpiozero==2.0.1`, `pigpio==1.78`, `pyorcasdk==1.1.0`, `PyYAML`
5) Run
   - Use `PYTHONPATH=src python -m orca_v1 --config configs/default.yaml`
6) Configuration
   - Explain `configs/default.yaml` keys at a high level (serial, motion, haptics, contact threshold, GPIO)
7) GPIO Triggers
   - AutoZero pin and Apply pin behavior
   - Debounce + lockout explanation
8) Logging
   - Where CSVs are stored; meaning of key fields and events
9) Troubleshooting
   - `No module named orca_v1` → need `PYTHONPATH=src` or editable install
   - `No module named yaml` → install `PyYAML`
   - pigpio daemon issues
10) Current Limitations (V1)
   - Validation stage; fault supervisor is basic; some policies conservative

Keep it concise, accurate, and runnable.

### Task B — Add missing comments/docstrings (high priority)
Go file-by-file under `src/orca_v1/` and do the following:

- Ensure every module has a clear module docstring.
- Ensure public functions have docstrings that describe:
  - What it does
  - Inputs/outputs
  - Safety notes if relevant
- Add inline comments only where they clarify tricky logic (avoid obvious comments).

**Important:** Do not “rewrite” code while commenting. No logic changes.

### Task C — Cross-check documentation accuracy (required)
After writing README and comments:
- Verify all commands match the repo layout (`src/` layout).
- Verify file names and module names are correct.
- Verify configuration keys match `config.py` and `configs/default.yaml`.

---

## Style guidelines
- Comments should be short, factual, and not speculative.
- Prefer docstrings over inline comments.
- Do not over-comment obvious Python.
- Use consistent terms:
  - “AutoZero”, “HOME”, “APPLY”, “HAPTIC bumper”, “CONTACT”, “RETURN”
  - “stream cache” for streaming reads

---

## Validation note
This repo is being validated via repeated apply cycles and CSV analysis. Documentation should encourage users to:
- run a single-cycle sanity check first
- then do 10–20 repeatability cycles
- inspect logs for `ENTER_HAPTIC`, `CONTACT_TRIGGER`, `HOME_REACHED`

---

## If you are uncertain
If something is ambiguous, do NOT guess. Instead:
- Add a brief note in README under “Known Issues / Assumptions”
- Or add a `TODO:` comment that clearly states what needs confirmation
But do not change code.
