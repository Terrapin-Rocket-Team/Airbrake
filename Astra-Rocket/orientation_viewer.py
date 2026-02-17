#!/usr/bin/env python3
"""
Telemetry-first Mahony-style visualizer.

Features:
- Requests telemetry header via serial command (default: CMD/HEADER)
- Parses TELEM/<header> and dynamically maps column indices
- Parses TELEM/<row> data lines using mapped columns
- Plots accel / gyro / mag time series
- Renders 3D body axes when quaternion columns are available

Usage:
  python orientation_viewer.py --port COM7
  python orientation_viewer.py --port /dev/ttyACM0 --baud 115200
"""

import argparse
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import serial
from serial.tools import list_ports


def pick_port() -> str:
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found.")

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} ({port.description})")

    while True:
        selection = input("Select port: ").strip()
        if selection.isdigit() and 0 <= int(selection) < len(ports):
            return ports[int(selection)].device
        print("Invalid selection.")


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    n = w * w + x * x + y * y + z * z
    if n <= 0.0:
        return np.eye(3)
    s = 2.0 / n

    wx = s * w * x
    wy = s * w * y
    wz = s * w * z
    xx = s * x * x
    xy = s * x * y
    xz = s * x * z
    yy = s * y * y
    yz = s * y * z
    zz = s * z * z

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def find_first_index(col_map: dict[str, int], keys: list[str]):
    for key in keys:
        idx = col_map.get(key)
        if idx is not None:
            return idx
    return None


def parse_header_line(line: str):
    if not line.startswith("TELEM/"):
        return None
    raw = line[6:].strip()
    if "," not in raw:
        return None
    parts = [p.strip() for p in raw.split(",")]
    if not parts:
        return None
    return {name: i for i, name in enumerate(parts)}, parts


def main() -> int:
    parser = argparse.ArgumentParser(description="Telemetry-based Mahony visualizer.")
    parser.add_argument("--port", help="Serial port (e.g., COM5).")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate.")
    parser.add_argument("--plot-hz", type=float, default=10.0, help="Plot update rate (Hz).")
    parser.add_argument("--max-points", type=int, default=300, help="Max points in each plot.")
    parser.add_argument(
        "--header-cmd",
        default="CMD/HEADER",
        help="Command sent to request telemetry header.",
    )
    parser.add_argument(
        "--header-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for header before continuing.",
    )
    args = parser.parse_args()

    port = args.port or pick_port()
    ser = serial.Serial(port, args.baud, timeout=0)
    time.sleep(1.5)
    ser.reset_input_buffer()

    # Ask FC for TELEM header.
    header_cmd = args.header_cmd.strip()
    if header_cmd:
        ser.write((header_cmd + "\n").encode("utf-8"))
        print(f"Sent header request: {header_cmd}")

    plt.ion()
    fig = plt.figure(figsize=(12, 8))

    ax_orient = fig.add_subplot(2, 2, 1, projection="3d")
    ax_orient.set_xlim(-1, 1)
    ax_orient.set_ylim(-1, 1)
    ax_orient.set_zlim(-1, 1)
    ax_orient.set_xlabel("X")
    ax_orient.set_ylabel("Y")
    ax_orient.set_zlabel("Z")
    ax_orient.set_title("Orientation (Quaternion From TELEM)")

    x_line, = ax_orient.plot([0, 1], [0, 0], [0, 0], color="r", linewidth=2, label="X")
    y_line, = ax_orient.plot([0, 0], [0, 1], [0, 0], color="g", linewidth=2, label="Y")
    z_line, = ax_orient.plot([0, 0], [0, 0], [0, 1], color="b", linewidth=2, label="Z")
    ax_orient.legend(loc="upper left")

    ax_acc = fig.add_subplot(2, 2, 2)
    ax_acc.set_title("Accel (m/s^2)")
    ax_acc.set_xlabel("Time (s)")
    ax_acc.set_ylabel("m/s^2")
    ax_acc.grid(True, alpha=0.3)

    ax_gyro = fig.add_subplot(2, 2, 3)
    ax_gyro.set_title("Gyro (rad/s)")
    ax_gyro.set_xlabel("Time (s)")
    ax_gyro.set_ylabel("rad/s")
    ax_gyro.grid(True, alpha=0.3)

    ax_mag = fig.add_subplot(2, 2, 4)
    ax_mag.set_title("Mag (uT)")
    ax_mag.set_xlabel("Time (s)")
    ax_mag.set_ylabel("uT")
    ax_mag.grid(True, alpha=0.3)

    status_text = fig.text(0.02, 0.01, "Waiting for TELEM header...", fontsize=9, family="monospace")

    window = max(50, args.max_points)
    times = deque(maxlen=window)
    acc_x = deque(maxlen=window)
    acc_y = deque(maxlen=window)
    acc_z = deque(maxlen=window)
    gyro_x = deque(maxlen=window)
    gyro_y = deque(maxlen=window)
    gyro_z = deque(maxlen=window)
    mag_x = deque(maxlen=window)
    mag_y = deque(maxlen=window)
    mag_z = deque(maxlen=window)

    acc_line_x, = ax_acc.plot([], [], color="r", label="Ax")
    acc_line_y, = ax_acc.plot([], [], color="g", label="Ay")
    acc_line_z, = ax_acc.plot([], [], color="b", label="Az")
    ax_acc.legend(loc="upper right")

    gyro_line_x, = ax_gyro.plot([], [], color="r", label="Gx")
    gyro_line_y, = ax_gyro.plot([], [], color="g", label="Gy")
    gyro_line_z, = ax_gyro.plot([], [], color="b", label="Gz")
    ax_gyro.legend(loc="upper right")

    mag_line_x, = ax_mag.plot([], [], color="r", label="Mx")
    mag_line_y, = ax_mag.plot([], [], color="g", label="My")
    mag_line_z, = ax_mag.plot([], [], color="b", label="Mz")
    ax_mag.legend(loc="upper right")

    plot_interval = 1.0 / max(0.5, args.plot_hz)
    last_plot = time.time()
    serial_buffer = bytearray()

    header_deadline = time.time() + max(0.1, args.header_timeout)
    last_header_request = time.time()
    col_map = None
    idx_time = None
    idx_ax = idx_ay = idx_az = None
    idx_gx = idx_gy = idx_gz = None
    idx_mx = idx_my = idx_mz = None
    idx_qw = idx_qx = idx_qy = idx_qz = None
    latest_quat = None

    while True:
        if not plt.fignum_exists(fig.number):
            break

        had_data = False
        waiting = ser.in_waiting
        if waiting:
            serial_buffer.extend(ser.read(waiting))
            had_data = True

        # Re-request header periodically until parsed.
        if col_map is None and header_cmd and (time.time() - last_header_request) > 1.0:
            ser.write((header_cmd + "\n").encode("utf-8"))
            last_header_request = time.time()

        while b"\n" in serial_buffer:
            raw_line, _, serial_buffer = serial_buffer.partition(b"\n")
            line = raw_line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parsed = parse_header_line(line)
            if parsed is not None:
                maybe_map, header_cols = parsed
                # Heuristic: treat as header when it includes common labels.
                if "State - Time (s)" in maybe_map or "State - PX (m)" in maybe_map:
                    col_map = maybe_map
                    idx_time = find_first_index(col_map, ["State - Time (s)", "Time (s)"])
                    idx_ax = find_first_index(
                        col_map,
                        [
                            "HITL_Accelerometer - Acc X (m/s^2)",
                            "BMI088 - Acc X (m/s^2)",
                            "State - AX (m/s/s)",
                            "State - AX (m/s^2)",
                        ],
                    )
                    idx_ay = find_first_index(
                        col_map,
                        [
                            "HITL_Accelerometer - Acc Y (m/s^2)",
                            "BMI088 - Acc Y (m/s^2)",
                            "State - AY (m/s/s)",
                            "State - AY (m/s^2)",
                        ],
                    )
                    idx_az = find_first_index(
                        col_map,
                        [
                            "HITL_Accelerometer - Acc Z (m/s^2)",
                            "BMI088 - Acc Z (m/s^2)",
                            "State - AZ (m/s/s)",
                            "State - AZ (m/s^2)",
                        ],
                    )
                    idx_gx = find_first_index(col_map, ["HITL_Gyroscope - Gyro X (rad/s)", "BMI088 - Gyro X (rad/s)"])
                    idx_gy = find_first_index(col_map, ["HITL_Gyroscope - Gyro Y (rad/s)", "BMI088 - Gyro Y (rad/s)"])
                    idx_gz = find_first_index(col_map, ["HITL_Gyroscope - Gyro Z (rad/s)", "BMI088 - Gyro Z (rad/s)"])
                    idx_mx = find_first_index(col_map, ["HITL_Magnetometer - Mag X (uT)", "MMC5603NJ - Mag X (uT)"])
                    idx_my = find_first_index(col_map, ["HITL_Magnetometer - Mag Y (uT)", "MMC5603NJ - Mag Y (uT)"])
                    idx_mz = find_first_index(col_map, ["HITL_Magnetometer - Mag Z (uT)", "MMC5603NJ - Mag Z (uT)"])
                    idx_qw = find_first_index(col_map, ["State - QW", "State - Quat W", "QW"])
                    idx_qx = find_first_index(col_map, ["State - QX", "State - Quat X", "QX"])
                    idx_qy = find_first_index(col_map, ["State - QY", "State - Quat Y", "QY"])
                    idx_qz = find_first_index(col_map, ["State - QZ", "State - Quat Z", "QZ"])

                    status_text.set_text(
                        f"Header parsed: {len(header_cols)} cols | "
                        f"t={idx_time} a=({idx_ax},{idx_ay},{idx_az}) "
                        f"g=({idx_gx},{idx_gy},{idx_gz}) m=({idx_mx},{idx_my},{idx_mz}) "
                        f"q=({idx_qw},{idx_qx},{idx_qy},{idx_qz})"
                    )
                    print(status_text.get_text())
                    continue

            if not line.startswith("TELEM/"):
                continue

            if col_map is None:
                # No header yet, ignore rows until map exists.
                continue

            payload = line[6:].strip()
            values = [v.strip() for v in payload.split(",")]

            try:
                if idx_time is not None and idx_time < len(values):
                    t = float(values[idx_time])
                else:
                    continue

                def get_float(idx):
                    if idx is None or idx >= len(values):
                        return None
                    try:
                        return float(values[idx])
                    except ValueError:
                        return None

                axv, ayv, azv = get_float(idx_ax), get_float(idx_ay), get_float(idx_az)
                gxv, gyv, gzv = get_float(idx_gx), get_float(idx_gy), get_float(idx_gz)
                mxv, myv, mzv = get_float(idx_mx), get_float(idx_my), get_float(idx_mz)
                qw, qx, qy, qz = get_float(idx_qw), get_float(idx_qx), get_float(idx_qy), get_float(idx_qz)

                if axv is not None and ayv is not None and azv is not None:
                    times.append(t)
                    acc_x.append(axv)
                    acc_y.append(ayv)
                    acc_z.append(azv)

                    gyro_x.append(0.0 if gxv is None else gxv)
                    gyro_y.append(0.0 if gyv is None else gyv)
                    gyro_z.append(0.0 if gzv is None else gzv)

                    mag_x.append(0.0 if mxv is None else mxv)
                    mag_y.append(0.0 if myv is None else myv)
                    mag_z.append(0.0 if mzv is None else mzv)

                if None not in (qw, qx, qy, qz):
                    latest_quat = (qw, qx, qy, qz)

            except Exception:
                continue

        # Warn if no header received in time, but keep running.
        if col_map is None and time.time() > header_deadline:
            status_text.set_text(
                f"No header yet. Retrying '{header_cmd}' every 1s. "
                "Ensure FC command router supports CMD/HEADER."
            )

        now = time.time()
        if now - last_plot >= plot_interval:
            last_plot = now

            if latest_quat is not None:
                r = quat_to_rot(*latest_quat)
                x_axis = r @ np.array([1.0, 0.0, 0.0])
                y_axis = r @ np.array([0.0, 1.0, 0.0])
                z_axis = r @ np.array([0.0, 0.0, 1.0])

                x_line.set_data([0, x_axis[0]], [0, x_axis[1]])
                x_line.set_3d_properties([0, x_axis[2]])
                y_line.set_data([0, y_axis[0]], [0, y_axis[1]])
                y_line.set_3d_properties([0, y_axis[2]])
                z_line.set_data([0, z_axis[0]], [0, z_axis[1]])
                z_line.set_3d_properties([0, z_axis[2]])

            acc_line_x.set_data(times, acc_x)
            acc_line_y.set_data(times, acc_y)
            acc_line_z.set_data(times, acc_z)
            gyro_line_x.set_data(times, gyro_x)
            gyro_line_y.set_data(times, gyro_y)
            gyro_line_z.set_data(times, gyro_z)
            mag_line_x.set_data(times, mag_x)
            mag_line_y.set_data(times, mag_y)
            mag_line_z.set_data(times, mag_z)

            for axis in (ax_acc, ax_gyro, ax_mag):
                axis.relim()
                axis.autoscale_view()

            plt.pause(0.001)

        if not had_data:
            time.sleep(0.001)

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

