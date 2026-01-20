#!/usr/bin/env python3
"""
Real-time Rocket Orientation Viewer

Connects to the flight computer via USB serial and visualizes:
1. Live 3D orientation of the rocket
2. Acceleration vectors (body-frame and earth-frame)
3. Accelerometer magnitude vs expected 9.81 m/s²
4. Mahony filter diagnostic data

This helps debug orientation filter issues like incorrect acceleration transformations.

Usage:
    python orientation_viewer.py COM3      # Windows
    python orientation_viewer.py /dev/ttyACM0  # Linux/Mac
"""

import serial
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform

class Arrow3D(FancyArrowPatch):
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)

class OrientationViewer:
    def __init__(self, serial_port, baud=115200):
        self.ser = serial.Serial(serial_port, baud, timeout=0.1)
        time.sleep(2)  # Wait for connection

        # Data storage
        self.accel_body = np.array([0.0, 0.0, 9.81])
        self.accel_earth = np.array([0.0, 0.0, 0.0])
        self.gyro = np.array([0.0, 0.0, 0.0])
        self.orientation_q = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z

        # Time series data for plotting
        self.times = []
        self.accel_body_mag = []
        self.accel_earth_mag = []
        self.max_points = 500

        # Parse telemetry header
        self.column_map = None
        self._find_header()

    def _find_header(self):
        """Look for TELEM header in startup messages"""
        print("Looking for telemetry header...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("TELEM/") and 'State - Time (s)' in line:
                    self._parse_header(line)
                    print(f"Found header with {len(self.column_map)} columns")
                    return
                elif line:
                    print(f"[FC] {line}")
        print("Warning: No header found, will look for it during runtime")

    def _parse_header(self, header_line):
        """Parse TELEM header to create column mapping"""
        header_str = header_line[6:].strip()
        columns = header_str.split(',')
        self.column_map = {}
        for idx, col_name in enumerate(columns):
            self.column_map[col_name.strip()] = idx

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion [w, x, y, z] to 3x3 rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

    def update_data(self):
        """Read and parse latest telemetry"""
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            # Check for header
            if line.startswith("TELEM/") and 'State - Time (s)' in line:
                self._parse_header(line)
                continue

            # Parse telemetry data
            if line.startswith("TELEM/") and self.column_map:
                data_str = line[6:].strip()
                values = data_str.split(',')

                try:
                    # Extract accelerometer data (body frame)
                    # Try HITL sensors first, then real BMI088 sensors
                    ax_idx = self.column_map.get('HITL_Accelerometer - Acc X (m/s^2)') or \
                             self.column_map.get('BMI088 - Acc X (m/s^2)') or \
                             self.column_map.get('BNO085Accel - Acc X (m/s^2)') or \
                             self.column_map.get('BMI088andLIS3MDL - Acc X (m/s^2)')
                    ay_idx = self.column_map.get('HITL_Accelerometer - Acc Y (m/s^2)') or \
                             self.column_map.get('BMI088 - Acc Y (m/s^2)') or \
                             self.column_map.get('BNO085Accel - Acc Y (m/s^2)') or \
                             self.column_map.get('BMI088andLIS3MDL - Acc Y (m/s^2)')
                    az_idx = self.column_map.get('HITL_Accelerometer - Acc Z (m/s^2)') or \
                             self.column_map.get('BMI088 - Acc Z (m/s^2)') or \
                             self.column_map.get('BNO085Accel - Acc Z (m/s^2)') or \
                             self.column_map.get('BMI088andLIS3MDL - Acc Z (m/s^2)')

                    if ax_idx is not None and ax_idx < len(values):
                        self.accel_body = np.array([
                            float(values[ax_idx]),
                            float(values[ay_idx]),
                            float(values[az_idx])
                        ])

                    # Extract gyroscope data
                    # Try HITL sensors first, then real BMI088 sensors
                    gx_idx = self.column_map.get('HITL_Gyroscope - Gyro X (rad/s)') or \
                             self.column_map.get('BMI088 - Gyro X (rad/s)') or \
                             self.column_map.get('BNO085Gyro - Gyro X (rad/s)') or \
                             self.column_map.get('BMI088andLIS3MDL - Gyro X (rad/s)')
                    gy_idx = self.column_map.get('HITL_Gyroscope - Gyro Y (rad/s)') or \
                             self.column_map.get('BMI088 - Gyro Y (rad/s)') or \
                             self.column_map.get('BNO085Gyro - Gyro Y (rad/s)') or \
                             self.column_map.get('BMI088andLIS3MDL - Gyro Y (rad/s)')
                    gz_idx = self.column_map.get('HITL_Gyroscope - Gyro Z (rad/s)') or \
                             self.column_map.get('BMI088 - Gyro Z (rad/s)') or \
                             self.column_map.get('BNO085Gyro - Gyro Z (rad/s)') or \
                             self.column_map.get('BMI088andLIS3MDL - Gyro Z (rad/s)')

                    if gx_idx is not None and gx_idx < len(values):
                        self.gyro = np.array([
                            float(values[gx_idx]),
                            float(values[gy_idx]),
                            float(values[gz_idx])
                        ])

                    # Extract state acceleration (earth frame, from Mahony filter)
                    state_ax = self.column_map.get('State - AX (m/s/s)')
                    state_ay = self.column_map.get('State - AY (m/s/s)')
                    state_az = self.column_map.get('State - AZ (m/s/s)')

                    if state_ax is not None and state_ax < len(values):
                        self.accel_earth = np.array([
                            float(values[state_ax]),
                            float(values[state_ay]),
                            float(values[state_az])
                        ])

                    # Get time for plotting
                    time_idx = self.column_map.get('State - Time (s)')
                    if time_idx is not None and time_idx < len(values):
                        t = float(values[time_idx])
                        self.times.append(t)
                        self.accel_body_mag.append(np.linalg.norm(self.accel_body))
                        self.accel_earth_mag.append(np.linalg.norm(self.accel_earth))

                        # Keep only recent data
                        if len(self.times) > self.max_points:
                            self.times.pop(0)
                            self.accel_body_mag.pop(0)
                            self.accel_earth_mag.pop(0)

                except (ValueError, IndexError) as e:
                    pass
            elif line and not line.startswith("TELEM/"):
                # Print non-telemetry messages
                print(f"[FC] {line}")

    def draw_rocket(self, ax):
        """Draw a simple rocket body"""
        # Rocket dimensions (body frame: Z-up is rocket axis)
        length = 2.0
        radius = 0.3

        # Get rotation matrix from quaternion
        R = self.quaternion_to_rotation_matrix(self.orientation_q)

        # Draw rocket body (cylinder along Z axis)
        theta = np.linspace(0, 2*np.pi, 20)
        z = np.array([0, length])

        # Cylinder points in body frame
        x_cyl = radius * np.outer(np.cos(theta), np.ones(len(z)))
        y_cyl = radius * np.outer(np.sin(theta), np.ones(len(z)))
        z_cyl = np.outer(np.ones(len(theta)), z)

        # Rotate to current orientation
        for i in range(len(theta)):
            for j in range(len(z)):
                point = np.array([x_cyl[i,j], y_cyl[i,j], z_cyl[i,j]])
                rotated = R @ point
                x_cyl[i,j], y_cyl[i,j], z_cyl[i,j] = rotated

        ax.plot_surface(x_cyl, y_cyl, z_cyl, alpha=0.3, color='gray')

        # Draw nose cone
        nose_height = 0.5
        nose_points = []
        for t in theta:
            base = R @ np.array([radius * np.cos(t), radius * np.sin(t), length])
            tip = R @ np.array([0, 0, length + nose_height])
            nose_points.append([base, tip])

        for i in range(len(nose_points)):
            p1 = nose_points[i][0]
            p2 = nose_points[i][1]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'k-', alpha=0.3)

    def draw_axes(self, ax):
        """Draw body-frame axes (X=red, Y=green, Z=blue)"""
        R = self.quaternion_to_rotation_matrix(self.orientation_q)

        # Axis vectors in body frame
        x_body = np.array([1, 0, 0])
        y_body = np.array([0, 1, 0])
        z_body = np.array([0, 0, 1])

        # Rotate to earth frame
        x_earth = R @ x_body
        y_earth = R @ y_body
        z_earth = R @ z_body

        # Draw arrows
        scale = 1.5
        arrow_x = Arrow3D(0, 0, 0, x_earth[0]*scale, x_earth[1]*scale, x_earth[2]*scale,
                         mutation_scale=20, lw=2, arrowstyle='-|>', color='red')
        arrow_y = Arrow3D(0, 0, 0, y_earth[0]*scale, y_earth[1]*scale, y_earth[2]*scale,
                         mutation_scale=20, lw=2, arrowstyle='-|>', color='green')
        arrow_z = Arrow3D(0, 0, 0, z_earth[0]*scale, z_earth[1]*scale, z_earth[2]*scale,
                         mutation_scale=20, lw=2, arrowstyle='-|>', color='blue')

        ax.add_artist(arrow_x)
        ax.add_artist(arrow_y)
        ax.add_artist(arrow_z)

        # Add labels
        ax.text(x_earth[0]*scale*1.2, x_earth[1]*scale*1.2, x_earth[2]*scale*1.2, 'X', color='red', fontsize=12, fontweight='bold')
        ax.text(y_earth[0]*scale*1.2, y_earth[1]*scale*1.2, y_earth[2]*scale*1.2, 'Y', color='green', fontsize=12, fontweight='bold')
        ax.text(z_earth[0]*scale*1.2, z_earth[1]*scale*1.2, z_earth[2]*scale*1.2, 'Z (Up)', color='blue', fontsize=12, fontweight='bold')

    def draw_acceleration_vectors(self, ax):
        """Draw acceleration vectors"""
        R = self.quaternion_to_rotation_matrix(self.orientation_q)

        # Body-frame acceleration (rotate to earth frame for visualization)
        accel_body_earth = R @ self.accel_body

        # Scale for visibility
        scale = 0.2

        # Draw body-frame accel (in earth frame for visualization) - CYAN
        if np.linalg.norm(accel_body_earth) > 0.1:
            arrow_body = Arrow3D(0, 0, 0,
                                accel_body_earth[0]*scale,
                                accel_body_earth[1]*scale,
                                accel_body_earth[2]*scale,
                                mutation_scale=20, lw=3, arrowstyle='-|>', color='cyan')
            ax.add_artist(arrow_body)
            ax.text(accel_body_earth[0]*scale*1.3,
                   accel_body_earth[1]*scale*1.3,
                   accel_body_earth[2]*scale*1.3,
                   f'Body Accel\n{np.linalg.norm(self.accel_body):.2f} m/s²',
                   color='cyan', fontsize=10, fontweight='bold')

        # Draw earth-frame accel (from Mahony filter) - MAGENTA
        if np.linalg.norm(self.accel_earth) > 0.1:
            arrow_earth = Arrow3D(0, 0, 0,
                                 self.accel_earth[0]*scale,
                                 self.accel_earth[1]*scale,
                                 self.accel_earth[2]*scale,
                                 mutation_scale=20, lw=3, arrowstyle='-|>', color='magenta')
            ax.add_artist(arrow_earth)
            ax.text(self.accel_earth[0]*scale*1.3,
                   self.accel_earth[1]*scale*1.3,
                   self.accel_earth[2]*scale*1.3,
                   f'Earth Accel\n{np.linalg.norm(self.accel_earth):.2f} m/s²',
                   color='magenta', fontsize=10, fontweight='bold')

def main():
    if len(sys.argv) < 2:
        print("Usage: python orientation_viewer.py <serial_port>")
        print("Example: python orientation_viewer.py COM3")
        print("         python orientation_viewer.py /dev/ttyACM0")
        sys.exit(1)

    port = sys.argv[1]

    print("===========================================")
    print("  Rocket Orientation Viewer")
    print("===========================================")
    print(f"Connecting to {port}...")

    try:
        viewer = OrientationViewer(port)
        print("Connected!")
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        sys.exit(1)

    # Create figure with subplots
    fig = plt.figure(figsize=(16, 8))

    # 3D orientation plot
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_xlabel('East (X)', fontsize=10)
    ax1.set_ylabel('North (Y)', fontsize=10)
    ax1.set_zlabel('Up (Z)', fontsize=10)
    ax1.set_title('Rocket Orientation (ENU Frame)', fontsize=12, fontweight='bold')
    ax1.set_xlim([-3, 3])
    ax1.set_ylim([-3, 3])
    ax1.set_zlim([0, 3])

    # Acceleration magnitude plot
    ax2 = fig.add_subplot(122)
    line_body, = ax2.plot([], [], 'c-', label='Body Accel Mag', linewidth=2)
    line_earth, = ax2.plot([], [], 'm-', label='Earth Accel Mag (Mahony)', linewidth=2)
    ax2.axhline(y=9.81, color='gray', linestyle='--', linewidth=1, label='Expected 9.81 m/s²')
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('Acceleration Magnitude (m/s²)', fontsize=10)
    ax2.set_title('Acceleration Magnitude Comparison', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # Text display for diagnostics
    info_text = fig.text(0.02, 0.95, '', fontsize=9, verticalalignment='top',
                        family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    def update(frame):
        # Read new data
        viewer.update_data()

        # Clear and redraw 3D plot
        ax1.clear()
        ax1.set_xlabel('East (X)', fontsize=10)
        ax1.set_ylabel('North (Y)', fontsize=10)
        ax1.set_zlabel('Up (Z)', fontsize=10)
        ax1.set_title('Rocket Orientation (ENU Frame)', fontsize=12, fontweight='bold')
        ax1.set_xlim([-3, 3])
        ax1.set_ylim([-3, 3])
        ax1.set_zlim([0, 3])

        # Draw rocket and axes
        # viewer.draw_rocket(ax1)  # Commented out for now, just show axes
        viewer.draw_axes(ax1)
        viewer.draw_acceleration_vectors(ax1)

        # Update acceleration plot
        if len(viewer.times) > 0:
            line_body.set_data(viewer.times, viewer.accel_body_mag)
            line_earth.set_data(viewer.times, viewer.accel_earth_mag)

            # Auto-scale
            ax2.set_xlim(min(viewer.times), max(viewer.times))
            all_mags = viewer.accel_body_mag + viewer.accel_earth_mag
            if all_mags:
                ax2.set_ylim(0, max(max(all_mags), 12))

        # Update diagnostic text
        body_mag = np.linalg.norm(viewer.accel_body)
        earth_mag = np.linalg.norm(viewer.accel_earth)
        gyro_mag = np.linalg.norm(viewer.gyro)

        info = f"""DIAGNOSTICS:
Body Accel:  [{viewer.accel_body[0]:7.2f}, {viewer.accel_body[1]:7.2f}, {viewer.accel_body[2]:7.2f}] m/s²  |Mag| = {body_mag:.3f}
Earth Accel: [{viewer.accel_earth[0]:7.2f}, {viewer.accel_earth[1]:7.2f}, {viewer.accel_earth[2]:7.2f}] m/s²  |Mag| = {earth_mag:.3f}
Gyro:        [{viewer.gyro[0]:7.3f}, {viewer.gyro[1]:7.3f}, {viewer.gyro[2]:7.3f}] rad/s  |Mag| = {gyro_mag:.3f}

Expected body accel on pad: ~9.81 m/s² (gravity + support force)
Expected earth accel on pad: ~0.00 m/s² (stationary, gravity removed)

Issue: Body accel shows {body_mag:.2f} m/s² but Earth accel shows {earth_mag:.2f} m/s²
"""
        info_text.set_text(info)

        return [ax1, ax2, info_text]

    # Animation
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    plt.tight_layout()
    plt.show()

    viewer.ser.close()

if __name__ == "__main__":
    main()
