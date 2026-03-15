python ProcessLookupError#!/usr/bin/env python3
"""
Plot FC telemetry CSV columns:
- X: Time - Seconds
- Y1: MotorDriver - Motor Angle
- Y2: DPS368 - Pres (hPa)
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


TIME_COL = "Time - Seconds"
ANGLE_COL = "MotorDriver - Motor Angle"
PRESSURE_COL = "DPS368 - Pres (hPa)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot motor angle and barometer pressure vs time from telemetry CSV.")
    parser.add_argument("csv_path", help="Path to telemetry CSV file")
    return parser.parse_args()


def to_float(value: str) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    times: list[float] = []
    angles: list[float] = []
    pressures: list[float] = []

    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise SystemExit("CSV has no header row.")

        required = [TIME_COL, ANGLE_COL, PRESSURE_COL]
        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            raise SystemExit(
                "Missing required columns: "
                + ", ".join(missing)
                + "\nAvailable columns: "
                + ", ".join(reader.fieldnames)
            )

        for row in reader:
            t = to_float(row.get(TIME_COL, ""))
            a = to_float(row.get(ANGLE_COL, ""))
            p = to_float(row.get(PRESSURE_COL, ""))
            if t is None or a is None or p is None:
                continue
            times.append(t)
            angles.append(a)
            pressures.append(p)

    if not times:
        raise SystemExit("No valid numeric rows found for requested columns.")

    fig, ax1 = plt.subplots(figsize=(11, 6))
    ax2 = ax1.twinx()

    line_angle, = ax1.plot(times, angles, color="#1f77b4", linewidth=1.8, label=ANGLE_COL)
    line_pressure, = ax2.plot(times, pressures, color="#d62728", linewidth=1.4, label=PRESSURE_COL)

    ax1.set_title(csv_path.name)
    ax1.set_xlabel(TIME_COL)
    ax1.set_ylabel(ANGLE_COL, color=line_angle.get_color())
    ax2.set_ylabel(PRESSURE_COL, color=line_pressure.get_color())

    ax1.grid(True, alpha=0.3)
    ax2.ticklabel_format(axis="y", style="plain", useOffset=False)

    lines = [line_angle, line_pressure]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, loc="best")

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
