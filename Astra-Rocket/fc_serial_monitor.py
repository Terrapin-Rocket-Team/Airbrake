#!/usr/bin/env python3
"""
FC serial monitor with:
- TELEM/ lines -> CSV
- non-TELEM lines -> event log
- live rolling plot (flap angle + barometer pressure)
- interactive command TX
- reconnect loop for missing/disconnected serial ports
"""

from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
from serial import SerialException


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def file_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def can_float(value: str) -> bool:
    try:
        float(value)
        return True
    except (ValueError, TypeError):
        return False


class FCMonitor:
    def __init__(self, port: str, baud: int, out_dir: Path, window_s: float, reconnect_delay_s: float) -> None:
        self.port = port
        self.baud = baud
        self.out_dir = out_dir
        self.window_s = window_s
        self.reconnect_delay_s = reconnect_delay_s

        self.stop_event = threading.Event()
        self.lock = threading.RLock()

        self.ser: Optional[serial.Serial] = None
        self.connected = False

        self.session_index = 0
        self.session_start_monotonic = time.monotonic()
        self.csv_path: Optional[Path] = None
        self.log_path: Optional[Path] = None
        self.csv_file = None
        self.log_file = None
        self.csv_writer: Optional[csv.writer] = None

        self.header: list[str] = []
        self.flap_idx: Optional[int] = None
        self.pressure_idx: Optional[int] = None
        self.time_idx: Optional[int] = None
        self.time_is_millis = False
        self.warned_missing_plot_columns = False

        self.times = deque()
        self.flap_angles = deque()
        self.pressures = deque()

        self.reader_thread: Optional[threading.Thread] = None
        self.input_thread: Optional[threading.Thread] = None
        self.animation: Optional[FuncAnimation] = None
        self.expecting_reconnect = False
        self.reconnect_requested = threading.Event()

        self.fig = None
        self.ax_flap = None
        self.ax_pressure = None
        self.line_flap = None
        self.line_pressure = None
        self.status_text = None

    def run(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.start_new_session("startup")

        self.reader_thread = threading.Thread(target=self.serial_reader_loop, name="serial-reader", daemon=True)
        self.reader_thread.start()

        self.input_thread = threading.Thread(target=self.user_input_loop, name="user-input", daemon=True)
        self.input_thread.start()

        self.setup_plot()
        try:
            plt.show()
        finally:
            self.stop_event.set()
            self.close_serial()
            self.close_files()
            if self.reader_thread:
                self.reader_thread.join(timeout=2.0)
            if self.input_thread:
                self.input_thread.join(timeout=0.2)

    def start_new_session(self, reason: str) -> None:
        with self.lock:
            self.close_files()
            self.session_index += 1
            self.session_start_monotonic = time.monotonic()
            self.times.clear()
            self.flap_angles.clear()
            self.pressures.clear()

            self.header = []
            self.flap_idx = None
            self.pressure_idx = None
            self.time_idx = None
            self.time_is_millis = False
            self.warned_missing_plot_columns = False

            stamp = file_stamp()
            base = f"{stamp}_session_{self.session_index:03d}"
            self.csv_path = self.out_dir / f"{base}_telem.csv"
            self.log_path = self.out_dir / f"{base}_events.log"
            self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
            self.log_file = self.log_path.open("w", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)

            self.log_event(f"SESSION START reason={reason}")
            print(f"[session] {self.session_index:03d} -> {self.csv_path.name}, {self.log_path.name}")

    def close_files(self) -> None:
        with self.lock:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
            if self.log_file is not None:
                self.log_file.flush()
                self.log_file.close()
                self.log_file = None

    def log_event(self, message: str) -> None:
        with self.lock:
            if self.log_file is not None:
                self.log_file.write(f"{now_iso()} {message}\n")
                self.log_file.flush()

    def open_serial_with_retry(self) -> Optional[serial.Serial]:
        while not self.stop_event.is_set():
            try:
                s = serial.Serial(self.port, self.baud, timeout=0.25)
                print(f"[serial] connected to {self.port} @ {self.baud}")
                self.log_event(f"SERIAL CONNECTED port={self.port} baud={self.baud}")
                return s
            except SerialException as exc:
                print(f"[serial] waiting for {self.port}: {exc}")
                time.sleep(self.reconnect_delay_s)
        return None

    def close_serial(self) -> None:
        with self.lock:
            if self.ser is not None:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                except SerialException:
                    pass
                self.ser = None
            self.connected = False

    def force_reconnect(self, reason: str) -> None:
        with self.lock:
            self.expecting_reconnect = True
        self.log_event(f"FORCE RECONNECT reason={reason}")
        self.reconnect_requested.set()
        with self.lock:
            serial_port = self.ser
        if serial_port is not None:
            try:
                # Wake blocking readline on platforms that support it.
                serial_port.cancel_read()
            except (AttributeError, SerialException, OSError):
                pass

    def serial_reader_loop(self) -> None:
        while not self.stop_event.is_set():
            serial_port = self.open_serial_with_retry()
            if serial_port is None:
                return

            with self.lock:
                self.ser = serial_port
                self.connected = True
                self.expecting_reconnect = False
                self.reconnect_requested.clear()

            try:
                while not self.stop_event.is_set():
                    if self.reconnect_requested.is_set():
                        self.log_event("SERIAL RECONNECT requested")
                        break

                    try:
                        raw = serial_port.readline()
                    except (SerialException, OSError, TypeError, AttributeError) as exc:
                        self.log_event(f"SERIAL READ ERROR reason={exc}")
                        break

                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    self.handle_rx_line(line)
            except (SerialException, OSError, TypeError, AttributeError) as exc:
                print(f"[serial] disconnected: {exc}")
                self.log_event(f"SERIAL DISCONNECTED reason={exc}")
            finally:
                self.close_serial()

    def user_input_loop(self) -> None:
        print("Type FC commands and press Enter. Example: AB/ANGLE 20, AB/SWEEP 1.5, AB/CRASH")
        print("Local commands: :help, :new, :quit")
        while not self.stop_event.is_set():
            try:
                cmd = input("> ").strip()
            except EOFError:
                self.stop_event.set()
                return
            except KeyboardInterrupt:
                self.stop_event.set()
                return

            if not cmd:
                continue

            if cmd in (":quit", ":exit"):
                self.stop_event.set()
                plt.close("all")
                return
            if cmd == ":help":
                print("FC: AB/ANGLE <deg>, AB/SWEEP <sec>, AB/SWEEP_STOP, AB/CRASH, ...")
                print("Local: :new (roll files), :quit")
                continue
            if cmd == ":new":
                self.start_new_session("manual_delimiter")
                continue

            self.send_command(cmd)

            if cmd.upper().startswith("AB/CRASH"):
                self.start_new_session("AB/CRASH delimiter")
                # FC crash/reboot commonly drops USB/UART link; force reconnect now.
                self.force_reconnect("AB/CRASH sent")

    def send_command(self, command: str) -> None:
        payload = command if command.endswith("\n") else f"{command}\n"
        self.log_event(f"TX {command}")
        with self.lock:
            serial_port = self.ser
            is_connected = self.connected and serial_port is not None and serial_port.is_open
        if not is_connected:
            print("[tx] serial not connected; command not sent")
            return

        try:
            assert serial_port is not None
            serial_port.write(payload.encode("utf-8"))
            serial_port.flush()
        except SerialException as exc:
            print(f"[tx] failed: {exc}")
            self.log_event(f"TX FAILED command={command} reason={exc}")

    def handle_rx_line(self, line: str) -> None:
        if line.startswith("TELEM/"):
            payload = line[len("TELEM/") :]
            self.handle_telem(payload)
            return

        print(line)
        self.log_event(f"RX {line}")

    def handle_telem(self, payload: str) -> None:
        cells = [cell.strip() for cell in payload.split(",")]
        if not cells:
            return

        if self.looks_like_header(cells):
            self.handle_header(cells)
            return

        with self.lock:
            if not self.header:
                self.header = [f"col_{i}" for i in range(len(cells))]
                self.csv_writer.writerow(["rx_time_iso"] + self.header)
                self.csv_file.flush()

            row = list(cells)
            if len(row) < len(self.header):
                row += [""] * (len(self.header) - len(row))
            elif len(row) > len(self.header):
                row = row[: len(self.header)]

            self.csv_writer.writerow([now_iso()] + row)
            self.csv_file.flush()

        self.update_plot_data(row)

    def looks_like_header(self, cells: list[str]) -> bool:
        if not cells:
            return False
        numeric_count = sum(1 for c in cells if can_float(c))
        return numeric_count < len(cells)

    def handle_header(self, cells: list[str]) -> None:
        with self.lock:
            if self.header and cells != self.header:
                self.start_new_session("telem_header_changed")

            self.header = list(cells)
            self.resolve_plot_columns()
            self.csv_writer.writerow(["rx_time_iso"] + self.header)
            self.csv_file.flush()

        self.log_event(f"TELEM HEADER {','.join(self.header)}")
        print(f"[telem] header columns={len(self.header)} flap_idx={self.flap_idx} pressure_idx={self.pressure_idx}")

    def resolve_plot_columns(self) -> None:
        lower = [h.lower() for h in self.header]

        def find_index(predicates: list[tuple[str, ...]]) -> Optional[int]:
            for idx, label in enumerate(lower):
                for required_words in predicates:
                    if all(word in label for word in required_words):
                        return idx
            return None

        self.flap_idx = find_index(
            [
                ("motor", "angle"),
                ("flap", "angle"),
                ("actuation", "angle"),
                ("actual", "angle"),
            ]
        )

        # Prefer explicit barometer pressure columns first.
        baro_pressure_idx = find_index(
            [
                ("baro", "pressure"),
                ("baro", "press"),
                ("baro", "pres"),
                ("barometer", "pressure"),
                ("barometer", "press"),
                ("barometer", "pres"),
            ]
        )
        if baro_pressure_idx is not None:
            self.pressure_idx = baro_pressure_idx
        else:
            # Fallback: any pressure-like column, with preference for hPa/Pa labels.
            self.pressure_idx = None
            for idx, label in enumerate(lower):
                if ("pressure" in label) or ("press" in label) or ("pres " in label) or ("pres(" in label) or ("hpa" in label) or (" pa" in label):
                    self.pressure_idx = idx
                    break

        # Time can be in seconds OR milliseconds depending on reporter labels.
        self.time_idx = find_index(
            [
                ("millis",),
                ("milliseconds",),
                ("timestamp",),
                ("time",),
                ("elapsed", "time"),
            ]
        )

        self.time_is_millis = False
        if self.time_idx is not None:
            time_label = lower[self.time_idx]
            self.time_is_millis = (
                ("millis" in time_label)
                or ("milliseconds" in time_label)
                or ("time (ms" in time_label)
                or (" ms)" in time_label)
                or ("[ms]" in time_label)
            )

    def update_plot_data(self, row: list[str]) -> None:
        with self.lock:
            flap_value = self.parse_optional_float(row, self.flap_idx)
            pressure_value = self.parse_optional_float(row, self.pressure_idx)
            t_seconds = self.extract_time_seconds(row)

            if t_seconds is None:
                t_seconds = time.monotonic() - self.session_start_monotonic

            if flap_value is None:
                flap_value = math.nan
            if pressure_value is None:
                pressure_value = math.nan

            if self.flap_idx is None or self.pressure_idx is None:
                if not self.warned_missing_plot_columns and self.header:
                    self.warned_missing_plot_columns = True
                    self.log_event("WARN plot columns not found for flap and/or pressure")
                    print("[plot] waiting for flap/pressure columns in TELEM header")

            self.times.append(t_seconds)
            self.flap_angles.append(flap_value)
            self.pressures.append(pressure_value)

            while self.times and (self.times[-1] - self.times[0]) > self.window_s:
                self.times.popleft()
                self.flap_angles.popleft()
                self.pressures.popleft()

    def extract_time_seconds(self, row: list[str]) -> Optional[float]:
        value = self.parse_optional_float(row, self.time_idx)
        if value is None:
            return None
        if self.time_is_millis:
            return value / 1000.0
        return value

    @staticmethod
    def parse_optional_float(row: list[str], idx: Optional[int]) -> Optional[float]:
        if idx is None or idx < 0 or idx >= len(row):
            return None
        token = row[idx].strip()
        if not token:
            return None
        try:
            return float(token)
        except ValueError:
            return None

    def setup_plot(self) -> None:
        self.fig, (self.ax_flap, self.ax_pressure) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
        self.line_flap, = self.ax_flap.plot([], [], linewidth=1.5, color="#1f77b4")
        self.line_pressure, = self.ax_pressure.plot([], [], linewidth=1.5, color="#d62728")

        self.ax_flap.set_ylabel("Flap Angle (deg)")
        self.ax_pressure.set_ylabel("Baro Pressure")
        self.ax_pressure.set_xlabel("Time (s)")
        self.ax_pressure.ticklabel_format(axis="y", style="plain", useOffset=False)
        self.ax_flap.grid(True, alpha=0.3)
        self.ax_pressure.grid(True, alpha=0.3)

        self.status_text = self.fig.text(0.01, 0.98, "", va="top", ha="left")
        self.fig.suptitle("FC Live Monitor")
        self.fig.tight_layout(rect=[0, 0, 1, 0.95])
        self.fig.canvas.mpl_connect("close_event", lambda _evt: self.stop_event.set())

        self.animation = FuncAnimation(self.fig, self.refresh_plot, interval=200, cache_frame_data=False)

    def refresh_plot(self, _frame: int):
        with self.lock:
            xs = list(self.times)
            ys_flap = list(self.flap_angles)
            ys_pressure = list(self.pressures)
            connected = self.connected
            session = self.session_index
            csv_name = self.csv_path.name if self.csv_path else "n/a"
            log_name = self.log_path.name if self.log_path else "n/a"

        self.line_flap.set_data(xs, ys_flap)
        self.line_pressure.set_data(xs, ys_pressure)

        if xs:
            x_max = xs[-1]
            x_min = max(0.0, x_max - self.window_s)
            if x_max <= x_min:
                x_max = x_min + 1.0
            self.ax_flap.set_xlim(x_min, x_max)
            self.ax_pressure.set_xlim(x_min, x_max)

            self.autoscale_y(self.ax_flap, ys_flap)
            self.autoscale_y(self.ax_pressure, ys_pressure)
        else:
            self.ax_flap.set_xlim(0, max(1.0, self.window_s))
            self.ax_pressure.set_xlim(0, max(1.0, self.window_s))

        status = (
            f"Port: {self.port} ({'CONNECTED' if connected else 'WAITING'}) | "
            f"Session: {session:03d} | CSV: {csv_name} | LOG: {log_name}"
        )
        self.status_text.set_text(status)
        return self.line_flap, self.line_pressure, self.status_text

    @staticmethod
    def autoscale_y(ax, values: list[float]) -> None:
        finite_values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
        if not finite_values:
            return
        y_min = min(finite_values)
        y_max = max(finite_values)
        if y_min == y_max:
            pad = 1.0 if y_min == 0 else abs(y_min) * 0.05
            ax.set_ylim(y_min - pad, y_max + pad)
            return
        pad = (y_max - y_min) * 0.1
        ax.set_ylim(y_min - pad, y_max + pad)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FC serial monitor with CSV/log split and live plotting.")
    parser.add_argument("--port", required=True, help="Serial port (example: COM8 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument(
        "--out-dir",
        default="fc_monitor_logs",
        help="Directory for output files (default: fc_monitor_logs)",
    )
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=15 * 30,
        help="Rolling plot window in seconds (default: 450)",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=1.0,
        help="Delay between open retries in seconds (default: 1.0)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    monitor = FCMonitor(
        port=args.port,
        baud=args.baud,
        out_dir=Path(args.out_dir),
        window_s=args.window_seconds,
        reconnect_delay_s=args.reconnect_delay,
    )
    monitor.run()


if __name__ == "__main__":
    main()
