import numpy as np
import matplotlib.pyplot as plt

a = np.array([0.0, 0.0, -9.8])  # inertial acceleration [m/s^2]
v = np.array([0.0, 0.0, 0.0])  # inertial velocity [m/s]
r = np.array([0.0, 0.0, 0.0])  # inertial position [m]
w = np.array([0.0, 0.0, 0.0])  # angular velocity [rad/s]

t = 0  # time [s]
lat, long = 0, 0

updateRate = 10  # [Hz]
timeStep = 1 / updateRate  # [s]
totalImpulse = 33664  # [Ns]
burnTime = 4.83  # [s]
rocketThrust = totalImpulse / burnTime  # [N]
launchTime = 10  # time of launch [s]

dryMass = 38.16  # [kg]
wetMass = 55.3  # [kg]
m = wetMass

CDr = 0.55
CDf = 0.95
flapArea = 0.00839
rocketArea = 0.01885
tilt_angle = np.deg2rad(1)  # Launch tilt angle (entered in degrees)

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
gyro_error = 0.02  # [rad/s]
mag_error = 50  # [micro T]
baro_error = 0.2  # [m]

def Propagate(flapAngle):
    global t, a, v, r, m, main_deployed, settling_timer, lat, long
    t += timeStep

    if t < launchTime:
        r[:] = 0.0
        v[:] = 0.0
        a[:] = [0.0, 0.0, -9.8]
        return

    density = getDensity(r[2])

    if t < burnTime + launchTime:
        m -= (wetMass - dryMass) / burnTime * timeStep
    else:
        m = dryMass

    speed = np.linalg.norm(v[[0, 2]])
    drag_force = 0.5 * density * (4 * CDf * flapArea * np.sin(np.deg2rad(flapAngle)) + CDr * rocketArea) * speed ** 2
    drag_accel = drag_force / m

    if t < burnTime + launchTime:
        thrust_accel = rocketThrust / m
        a[0] = thrust_accel * np.sin(tilt_angle) - drag_accel * np.sin(tilt_angle)
        a[2] = thrust_accel * np.cos(tilt_angle) - drag_accel * np.cos(tilt_angle) - 9.8
    elif v[2] < 0 and r[2] < main_deployment:
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
        a[2] = -0.5 / m * density * (drogue_Cd * drogue_area) * abs(v[2]) * v[2] - 9.8
        a[0] = 0
    else:
        a[0] = -drag_accel * np.sin(tilt_angle)
        a[2] = -drag_accel * np.cos(tilt_angle) - 9.8

    v += timeStep * a
    r += timeStep * v + 0.5 * a * timeStep ** 2
    lat, long = getLatLong(r)
    
    return


def getDensity(h):
    """
    Calculate air density given altitude h (in meters ASL).
    
    :param h: Altitude above sea level (m)
    :return: Air density (kg/m³)
    """
    # Constants
    R = 8.31446   # Universal gas constant (J/(mol·K))
    M = 0.0289652 # Molar mass of air (kg/mol)
    L = 0.0065    # Temperature lapse rate in the troposphere (K/m)

    p0 = 101325   # Ground-level pressure (Pa)
    T0 = 288.15   # Ground-level temperature (K)
    
    # Density calculation
    density = (p0 * M) / (R * T0) * np.power((1 - L * h / T0), ((9.8 * M / (R * L)) - 1))

    return density

def getPressure(h):

    # Constants
    R = 8.31446   # Universal gas constant (J/(mol·K))
    M = 0.0289652 # Molar mass of air (kg/mol)
    g0 = 9.81     # [m/s^2]
    Pb = 101325   # [Pa]
    Lb = -0.0065  # Temperature lapse rate in the troposphere (K/m)
    Tb = 288      # [K]
    h0 = 0        # [m]

    pressure = Pb * np.power((1 + (Lb/Tb) * (h - h0)), (-g0*M/(R*Lb)))

    return pressure

def getTemperature(h):

    # Constants
    R = 287   # Universal gas constant (J/(mol·K))

    temperature = getPressure(h)/(R*getDensity(h)) # ideal gas law
    temperature = temperature - 273.15 # kelvin to C

    return temperature

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
    time_data, altitude_data, x_data = [], [], []

    # Simulate flight
    while r[2] >= 0:
        Propagate(flap_angle)
        time_data.append(t)
        altitude_data.append(r[2])
        x_data.append(r[0])

    return np.array(time_data), np.array(altitude_data), np.array(x_data)

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Run simulation for a chosen flap angle
    flap_angle = 0
    time_data, altitude_data, x_data = simulate_flight(flap_angle)

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



