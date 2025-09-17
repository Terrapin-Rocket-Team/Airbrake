import numpy as np

# Constants
gamma = 1.4                # Ratio of specific heats for air
R = 287.05                 # Specific gas constant for air in J/(kg·K)

def air_properties(altitude):
    if altitude < 11000:  # Troposphere
        T = 288.15 - 0.0065 * altitude  # Temperature (K)
        P = 101325 * (T / 288.15) ** (9.80665 / (R * -0.0065))  # Pressure (Pa)
    else:  # Simple model beyond troposphere (constant T)
        T = 216.65
        P = 22632.1 * np.exp(-9.80665 * (altitude - 11000) / (R * T))
    rho = P / (R * T)  # Density (kg/m³)
    a = np.sqrt(gamma * R * T)  # Speed of sound (m/s)
    return rho, a

def drag_force_from_mach(mach, area, Cd, altitude=0):
    """
    Calculate the drag force on a surface given Mach number, area, and drag coefficient.

    Parameters:
        mach (float): Mach number of the object.
        area (float): Reference area in m².
        Cd (float): Drag coefficient (unitless).
        altitude (float): Altitude in meters (default is 0 m, sea level).

    Returns:
        float: Drag force in Newtons (N).
    """

    rho, a = air_properties(altitude)
    velocity = mach * a  # Velocity (m/s)

    # Drag force equation
    drag_force = 0.5 * rho * velocity**2 * Cd * area
    return drag_force