import numpy as np
import matplotlib.pyplot as plt

a = [0,0,-9.8] # inertial acceleration [m/s^2]
v = [0,0,0] # inertial velocity [m/s]
r = [0,0,0] # inertial position [m]
# o = [0,0,0,1]
w = [0,0,0] # angular velocity [rad/s]

t = 0 # time [s]

x = 0; y = 1; z = 2
lat = 0
long = 0

updateRate = 10 # [Hz]
timeStep = 1/updateRate # [s]
totalImpulse = 32000 # [Ns]
burnTime = 4.5 # [s]
rocketThrust = totalImpulse/burnTime # [N]
launchTime = 10 # time of launch [s] (necessary for the barometer to settle)

dryMass = 39; # [kg]
wetMass = 56.25; # [kg]
m = wetMass

CDr = .55
CDf = .95
flapArea = 0.00839
rocketArea = 0.01885

tilt_angle = np.deg2rad(0)  # Launch tilt angle in radians

main_deployment = 304.8 # [m] (1000 ft)
main_area = 11.9845 # [m^2] (30.5 ft)
main_Cd = 2.92 # [~]
drogue_area = 0.29186 # [m^2] (24 in)
drogue_Cd = 1.16 # [~]

main_settling_time = 2.0  # seconds for the parachute to settle
main_deployed = False
settling_timer = 0.0

def Propagate(flapAngle):
    # flapAngle in degrees
    flapAngle = np.deg2rad(flapAngle)

    global t, a, v, r, m, main_deployed, settling_timer, lat, long
    t += timeStep

    if t < launchTime:
        r[x] = 0;r[y] = 0;r[z] = 0
        v[x] = 0;v[y] = 0;v[z] = 0
        a[x] = 0;a[y] = 0;a[z] = -9.8
        return

    density = getDensity(r[z])

    #update Mass
    if t < burnTime + launchTime:
        m -= (wetMass - dryMass)/burnTime*timeStep
    else:
        m = dryMass

    # Compute aerodynamic drag force
    speed = np.sqrt(v[x]**2 + v[z]**2)
    drag_force = 0.5 * density * (4*CDf*flapArea*np.sin(flapAngle) + CDr*rocketArea) * speed**2
    drag_accel = drag_force / m
    
    #update Acceleration
    if t < burnTime + launchTime: # Motor Acceleration (Boost Phase)
        thrust_accel = rocketThrust / m
        a[x] = thrust_accel * np.sin(tilt_angle) - drag_accel * np.sin(tilt_angle)
        a[z] = thrust_accel * np.cos(tilt_angle) - drag_accel * np.cos(tilt_angle) - 9.8
    elif v[z] < 0 and r[z] < main_deployment: # Main deployment
        a[x] = 0
        if not main_deployed:
            main_deployed = True
            settling_timer = main_settling_time

        if settling_timer > 0:
            a[z] = -0.5 / m * density * (main_Cd * main_area) * np.sqrt(v[z] ** 2) * v[z] * 0.5  # Reduced force during settling
            settling_timer -= timeStep
        else:
            a[z] = -0.5 / m * density * (main_Cd * main_area) * np.sqrt(v[z] ** 2) * v[z] - 9.8
    elif v[z] < 0: # Drogue Deployment
        a[z] = -0.5/m*density*(drogue_Cd*drogue_area)*np.sqrt(v[z]*v[z])*v[z] - 9.8
        a[x] = 0
    else: # Coast Phase
        a[x] = -drag_accel * np.sin(tilt_angle)
        a[z] = -drag_accel * np.cos(tilt_angle) - 9.8

    #update Velocity
    v[x] = v[x] + timeStep*a[x]
    v[y] = v[y] + timeStep*a[y]
    v[z] = v[z] + timeStep*a[z]

    #update Position
    r[x] = r[x] + timeStep*v[x] + .5*a[x]*timeStep**2
    r[y] = r[y] + timeStep*v[y] + .5*a[y]*timeStep**2
    r[z] = r[z] + timeStep*v[z] + .5*a[z]*timeStep**2
    lat, long = getLatLong(r)

    #update orientation
    # o[w] = sqrt(v[x]^2 + v[y]^2 + v[z]^2)
    # o[x] = v[x]/o[w]
    # o[y] = v[y]/o[w]
    # o[z] = v[z]/o[w]

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
    R_EARTH = 6378137  # Earth's radius in meters (WGS-84)

    # Compute latitude
    lat = np.arcsin(r[z] / np.sqrt(r[x]**2 + r[y]**2 + r[z]**2))

    # Compute longitude
    long = np.arctan2(r[y], r[x])

    # Convert to degrees
    lat = np.degrees(lat)
    long = np.degrees(long)

    return lat, long



### Test the propagater ###

def simulate_flight(flap_angle):
    """Runs the simulation until the rocket returns to the ground."""
    global t, r, v, a, m
    t = 0
    r = [0, 0, 0]
    v = [0, 0, 0]
    a = [0, 0, -9.8]
    m = wetMass

    time_data = []
    altitude_data = []
    x_data = []

    while r[z] >= 0:
        Propagate(flap_angle)
        time_data.append(t)
        altitude_data.append(r[z])
        x_data.append(r[x])

    return time_data, altitude_data, x_data

if __name__ == "__main__":
    # Run simulation for a chosen flap angle
    time_data, altitude_data, x_data = simulate_flight(flap_angle=0)
    print(max(altitude_data))

    import matplotlib.pyplot as plt

    # Create a figure with two subplots
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Plot altitude vs. time
    axes[0].plot(time_data, altitude_data, label='Altitude (m)')
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Altitude (m)')
    axes[0].set_title('Rocket Altitude Over Time')
    axes[0].legend()
    axes[0].grid()

    # Plot flight path (altitude vs. x position)
    axes[1].plot(x_data, altitude_data, label='Trajectory')
    axes[1].set_xlabel('X Position (m)')
    axes[1].set_ylabel('Altitude (m)')
    axes[1].set_title('Rocket Flight Path')
    axes[1].legend()
    axes[1].grid()

    # Adjust layout and show the plots
    plt.tight_layout()
    plt.show()


