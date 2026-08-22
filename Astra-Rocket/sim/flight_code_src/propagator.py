import numpy as np
import matplotlib.pyplot as plt
from drag import total_drag_coefficient
import csv
import time

from ambiance import Atmosphere


a = np.array([0.0, 0.0, -9.8])  # inertial acceleration [m/s^2]
v = np.array([0.0, 0.0, 0.0])  # inertial velocity [m/s]
r = np.array([0.0, 0.0, 0.0])  # inertial position [m]
w = np.array([0.0, 0.0, 0.0])  # angular velocity [rad/s]

t = 0  # time [s]
lat, long = 0, 0

updateRate = 10  # [Hz]
timeStep = 1 / updateRate  # [s]
totalImpulse = 33417 # [Ns]
burnTime = 9.6 # [s]
rocketThrust = totalImpulse / burnTime  # [N]
launchTime = 5  # time of launch [s]
surface_roughness = 5e-6 # [m]
wetMass = 124 / 2.2 # [kg]
dryMass = wetMass - (37.47 / 2.2) # [kg]
m = wetMass

CDr = 0.47
CDf = 0.908
flapArea = 0.00987
rocket_diameter = 0.157 # [m]
rocket_body_area = np.pi * (rocket_diameter/2)**2 
rocket_fin_area = 0.00150839 * 4 # [m^2] (fin area = 0.0079375 m^2, 4 fins)
rocket_fillet_area = 0.00015775 * 8 # [m^2] (fillet area = 0.00015775 m^2, 8 fillets)
rocket_area = rocket_body_area + rocket_fin_area + rocket_fillet_area
print(rocket_area)
rocket_length = 13.0 / 3.28 # [m] (13 ft)
tilt_angle = np.deg2rad(0)  # Launch tilt angle (entered in degrees)
ground_altitude = 912 # [m]
atmosphere = Atmosphere(ground_altitude)
pressure_error_gain = 0.489  # [m/hPa]
reported_height_agl = 0.0
reported_height_error = 0.0
reported_pressure_hpa = float(np.asarray(atmosphere.pressure).reshape(-1)[0] / 100.0)

main_deployment = 304.8  # [m]
main_area = 11.9845  # [m^2]
main_Cd = 2.92
drogue_area = 0.29186  # [m^2]
drogue_Cd = 1.16

main_settling_time = 2.0  # seconds
main_deployed = False
settling_timer = 0.0

# Errors
accel_error = 0.1  # [m/s^2]
gyro_error = 0.1  # [rad/s]
mag_error = 50  # [micro T]
baro_error = 0.2  # [m]
br_baro_error = 0.5  # [m]
br_accel_error = 0.05  # [m/s^2]
br_gryo_error = 0.01  # [rad/s]


def _scalar(value) -> float:
    return float(np.asarray(value, dtype=float).reshape(-1)[0])


def _update_reported_pressure(flap_angle_deg: float) -> None:
    global reported_height_agl, reported_height_error, reported_pressure_hpa

    real_height_agl = max(0.0, float(r[2]))
    density = max(0.0, _scalar(atmosphere.density))
    airspeed = float(np.linalg.norm(v))
    theta_rad = np.deg2rad(flap_angle_deg)
    dynamic_pressure_hpa = 0.5 * density * airspeed ** 2 / 100.0

    reported_height_error = pressure_error_gain * dynamic_pressure_hpa * np.sin(theta_rad)
    reported_height_agl = max(0.0, real_height_agl + reported_height_error)

    reported_altitude_asl = max(0.0, ground_altitude + reported_height_agl)
    reported_pressure_hpa = _scalar(Atmosphere(reported_altitude_asl).pressure) / 100.0

def Propagate(flapAngle):
    global t, a, v, r, m, main_deployed, settling_timer, lat, long, atmosphere
    t += timeStep

    if t < launchTime:
        r[:] = 0.0
        v[:] = 0.0
        a[:] = [0.0, 0.0, -9.8]
        atmosphere = Atmosphere(ground_altitude)
        _update_reported_pressure(flapAngle)
        return

    atmosphere = Atmosphere(r[2]+ground_altitude)
    # Atmosphere backends often expose 1-element numpy arrays; normalize to scalars.
    density = float(np.asarray(atmosphere.density).reshape(-1)[0])
    dyn_viscosity = float(np.asarray(atmosphere.dynamic_viscosity).reshape(-1)[0])
    speed_of_sound = float(np.asarray(atmosphere.speed_of_sound).reshape(-1)[0])

    if t < burnTime + launchTime:
        m -= (wetMass - dryMass) / burnTime * timeStep
    else:
        m = dryMass

    speed = np.linalg.norm(v[[0, 2]])
    reynolds = density * speed * rocket_diameter / dyn_viscosity
    Cdr = 0.01168 / rocket_area #total_drag_coefficient(reynolds, speed / speed_of_sound, surface_roughness, rocket_length)
    drag_force = 0.5 * density * (4 * CDf * flapArea * np.sin(np.deg2rad(flapAngle)) + Cdr * rocket_area) * speed ** 2 # 
    drag_accel = drag_force / m

    if t < burnTime + launchTime:
        # Boost Phase
        thrust_accel = rocketThrust / m
        a[0] = thrust_accel * np.sin(tilt_angle) - drag_accel * np.sin(tilt_angle)
        a[2] = thrust_accel * np.cos(tilt_angle) - drag_accel * np.cos(tilt_angle) - 9.8
    elif v[2] < 0 and r[2] < main_deployment:
        # Main Deployment Phase
        a[0] = 0
        if not main_deployed:
            main_deployed = True
            settling_timer = main_settling_time
        if settling_timer > 0:
            a[2] = -0.5 / m * density * (main_Cd * main_area) * abs(v[2]) * v[2] * 0.5
            settling_timer -= timeStep
        else:
            a[2] = -0.5 / m * density * (main_Cd * main_area) * abs(v[2]) * v[2] - 9.8
    elif v[2] < 0:
        # Drogue Deployment Phase
        a[2] = -0.5 / m * density * (drogue_Cd * drogue_area) * abs(v[2]) * v[2] - 9.8
        a[0] = 0
    else:
        # Coasting Phase
        a[0] = -drag_accel * np.sin(tilt_angle)
        a[2] = -drag_accel * np.cos(tilt_angle) - 9.8

    v += timeStep * a
    r += timeStep * v + 0.5 * a * timeStep ** 2
    atmosphere = Atmosphere(max(0.0, r[2] + ground_altitude))
    _update_reported_pressure(flapAngle)
    lat, long = getLatLong(r)
    
    return


def getLatLong(r):
    """
    Compute latitude and longitude from inertial position vector.
    
    :param r: Inertial position vector [x, y, z] (meters)
    :return: (latitude, longitude) in degrees
    """
    R_EARTH = 6378137 # [m]
    lat = np.degrees(np.arcsin(r[2] / np.linalg.norm(r)))
    long = np.degrees(np.arctan2(r[1], r[0]))
    return lat, long

def threeDSingleAxisRotation(angle: float, axis: int) -> np.ndarray:
    """
    Returns the rotation matrix for a given angle and axis of rotation.

    Parameters:
    angle (float): The angle of rotation in radians.
    axis (int): The axis of rotation (1 for x-axis, 2 for y-axis, 3 for z-axis).

    Returns:
    np.ndarray: The 3x3 rotation matrix.
    """
    
    if axis == 1:
        return np.array([[1, 0, 0], 
                         [0, np.cos(angle), np.sin(angle)], 
                         [0, -np.sin(angle), np.cos(angle)]])
    elif axis == 2:
        return np.array([[np.cos(angle), 0, -np.sin(angle)], 
                         [0, 1, 0], 
                         [np.sin(angle), 0, np.cos(angle)]])
    elif axis == 3:
        return np.array([[np.cos(angle), np.sin(angle), 0], 
                         [-np.sin(angle), np.cos(angle), 0], 
                         [0, 0, 1]])
    else:
        raise ValueError("Invalid axis. Axis must be 0, 1, or 2.")

def interial2Body(vector, angle):
    vector_interial = np.array(vector)
    vector_body = threeDSingleAxisRotation(angle, 2) @ vector_interial

    return vector_body.tolist()

def gaussian_noise_generator(data, sigma):
    """
    Adds Gaussian noise to input data.

    Parameters:
    - data: The original data (can be a NumPy array or list).
    - sigma: The standard deviation of the Gaussian noise.

    Returns:
    - noisy_data: Data with added Gaussian noise.
    """
    data = np.array(data)  # Ensure input is a NumPy array
    noise = sigma * np.random.randn(*data.shape)  # Generate Gaussian noise
    noisy_data = data + noise  # Add noise to data

    return noisy_data


### Test the propagater ###

def simulate_flight(flap_angle):
    """Runs the simulation until the rocket returns to the ground."""
    global t, r, v, a, m

    # Initialize state variables using NumPy arrays
    t = 0
    r = np.array([0.0, 0.0, 0.0])  # Position (x, y, z)
    v = np.array([0.0, 0.0, 0.0])  # Velocity (x, y, z)
    a = np.array([0.0, 0.0, -9.8])  # Acceleration (x, y, z)
    m = wetMass  # Initial mass

    # Data storage
    data = []

    # Simulate flight
    while r[2] >= 0:
        Propagate(flap_angle)
        data.append([
            t,
            r[0], r[1], r[2],
            v[0], v[1], v[2],
            a[0], a[1], a[2]
        ])

    return np.array(data)

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Run simulation for a chosen flap angle
    flap_angle = 0
    data = simulate_flight(flap_angle)

    # Save to CSV
    with open("flight_data.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "Time (s)",
            "Pos X (m)", "Pos Y (m)", "Pos Z (m)",
            "Vel X (m/s)", "Vel Y (m/s)", "Vel Z (m/s)",
            "Acc X (m/s²)", "Acc Y (m/s²)", "Acc Z (m/s²)"
        ])
        writer.writerows(data)

    # Extract for plotting
    time_data = data[:, 0]
    altitude_data = data[:, 3]  # z
    x_data = data[:, 1]         # x

    # Print maximum altitude reached
    max_altitude = np.max(altitude_data)
    print(f"Maximum Altitude: {max_altitude:.2f} m")

    # Create a figure with two subplots
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Plot altitude vs. time
    axes[0].plot(time_data, altitude_data, label="Altitude (m)", color="b")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].set_title("Rocket Altitude Over Time")
    axes[0].legend()
    axes[0].grid(True, linestyle="--", alpha=0.7)

    # Plot flight path (altitude vs. x position)
    axes[1].plot(x_data, altitude_data, label="Trajectory", color="r")
    axes[1].set_xlabel("X Position (m)")
    axes[1].set_ylabel("Altitude (m)")
    axes[1].set_title("Rocket Flight Path")
    axes[1].legend()
    axes[1].grid(True, linestyle="--", alpha=0.7)

    # Adjust layout and show the plots
    plt.tight_layout()
    plt.show()



