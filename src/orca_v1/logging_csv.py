# src/orca_v1/logging_csv.py
"""
logging_csv.py (V1)

CSV logger aligned with your current test script columns:
  epoch_s, phase, elapsed_s, position_um, velocity_um_s,
  force_raw_mN, baseline_mN, delta_mN, event

Design goals:
- Zero dependencies (stdlib only)
- Creates log directory automatically
- Safe close (context manager)
- Minimal friction: one call per loop tick
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TextIO
import csv
from datetime import datetime
import os


CSV_COLUMNS = [
    "epoch_s",
    "phase",
    "elapsed_s",
    "position_um",
    "velocity_um_s",
    "force_raw_mN",
    "baseline_mN",
    "delta_mN",
    "event",
]


@dataclass(frozen=True)
class CsvLogConfig:
    log_dir: str = "logs"
    prefix: str = "apply_kin_then_haptic"
    include_header: bool = True
    # If you want UTC timestamps for filenames, set True later
    use_utc_for_filename: bool = False


class CsvLogger:
    """
    Usage:

        logger = CsvLogger(CsvLogConfig(log_dir="logs", prefix="apply_kin_then_haptic"))
        logger.open()
        logger.log(... each tick ...)
        logger.close()

    Or:

        with CsvLogger(cfg).open_ctx() as log:
            log.log(...)
    """

    def __init__(self, cfg: CsvLogConfig):
        self._cfg = cfg
        self._file: Optional[TextIO] = None
        self._writer: Optional[csv.writer] = None
        self._path: Optional[Path] = None

    @property
    def path(self) -> Optional[Path]:
        """Full path to the current CSV file (after open)."""
        return self._path

    def open(self) -> "CsvLogger":
        """Create log directory, open a new CSV file, write header."""
        if self._file is not None:
            return self  # already open

        log_dir = Path(self._cfg.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)

        ts = datetime.utcnow() if self._cfg.use_utc_for_filename else datetime.now()
        filename = f"{self._cfg.prefix}_{ts.strftime('%Y%m%d_%H%M%S')}.csv"
        path = log_dir / filename

        # newline="" is important for csv on all platforms
        f = open(path, "w", newline="", encoding="utf-8")
        w = csv.writer(f)

        if self._cfg.include_header:
            w.writerow(CSV_COLUMNS)

        self._file = f
        self._writer = w
        self._path = path
        return self

    def close(self) -> None:
        """Flush and close the CSV file (safe to call multiple times)."""
        if self._file is None:
            return
        try:
            self._file.flush()
            os.fsync(self._file.fileno())
        except Exception:
            # Best-effort: don't crash on fsync failures
            pass
        try:
            self._file.close()
        finally:
            self._file = None
            self._writer = None
            self._path = None

    # ---- context manager ----
    def __enter__(self) -> "CsvLogger":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ---- logging ----
    def log(
        self,
        *,
        epoch_s: float,
        phase: str,
        elapsed_s: float,
        position_um: int,
        velocity_um_s: float,
        force_raw_mN: int,
        baseline_mN: Optional[int] = None,
        event: str = "",
    ) -> None:
        """
        Append a row to CSV.

        - delta_mN is computed if baseline_mN is provided
        - numeric formatting mirrors your current usage (elapsed/epoch with fixed decimals)
        """
        if self._writer is None:
            raise RuntimeError("CsvLogger is not open(). Call open() before log().")

        delta = None if baseline_mN is None else (force_raw_mN - baseline_mN)

        row = [
            f"{epoch_s:.6f}",
            phase,
            f"{elapsed_s:.6f}",
            int(position_um),
            f"{float(velocity_um_s):.1f}",
            int(force_raw_mN),
            "" if baseline_mN is None else int(baseline_mN),
            "" if delta is None else int(delta),
            event,
        ]
        self._writer.writerow(row)

    def flush(self) -> None:
        """Flush buffered rows to disk (optional; can be called periodically)."""
        if self._file is None:
            return
        try:
            self._file.flush()
        except Exception:
            pass
