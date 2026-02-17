# Much of these functions can be found here: https://gist.github.com/gusgordon/3fa0a80e767a34ffb8b112c8630c5484

import numpy as np
from scipy.integrate import odeint


def temp_to_sos(T):
    # Speed of sound in dry air given temperature in K
    return 20.05 * T**0.5


def taylor_maccoll(y, theta, gamma=1.4):
    # Taylor-Maccoll function
    # Source: https://www.grc.nasa.gov/www/k-12/airplane/coneflow.html
    v_r, v_theta = y
    dydt = [
        v_theta,
        (v_theta ** 2 * v_r - (gamma - 1) / 2 * (1 - v_r ** 2 - v_theta ** 2) * (2 * v_r + v_theta / np.tan(theta))) / ((gamma - 1) / 2 * (1 - v_r ** 2 - v_theta ** 2) - v_theta ** 2) 
    ]
    return dydt


def oblique_shock(theta, Ma, T, p, rho, gamma=1.4):
    """
    Computes the weak oblique shock resulting from supersonic
    flow impinging on a wedge in 2 dimensional flow.
    
    Inputs:
     - theta is the angle of the wedge in radians.
     - Ma, T, p, and rho are the Mach number, temperature (K),
       pressure (Pa), and density (kg/m^3) of the flow.
     - gamma is the ratio of specific heats. Defaults
       to air's typical value of 1.4.
    
    Returns:
     - shock angle in radians
     - resultant flow direction in radians
     - respectively, Mach number, temperature, pressure, density,
       and velocity components downstream of shock.
    
    Source: https://www.grc.nasa.gov/WWW/K-12/airplane/oblique.html
    """
    x = np.tan(theta)
    for B in np.arange(1, 500) * np.pi/1000:
        r = 2 / np.tan(B) * (Ma**2 * np.sin(B)**2 - 1) / (Ma**2 * (gamma + np.cos(2 * B)) + 2)
        if r > x:
            break
    cot_a = np.tan(B) * ((gamma + 1) * Ma ** 2 / (2 * (Ma ** 2 * np.sin(B) ** 2 - 1)) - 1)
    a = np.arctan(1 / cot_a)

    Ma2 = 1 / np.sin(B - theta) * np.sqrt((1 + (gamma - 1)/2 * Ma**2 * np.sin(B)**2) / (gamma * Ma**2 * np.sin(B)**2 - (gamma - 1)/2))

    h = Ma ** 2 * np.sin(B) ** 2
    T2 = T * (2 * gamma * h - (gamma - 1)) * ((gamma - 1) * h + 2) / ((gamma + 1) ** 2 * h)
    p2 = p * (2 * gamma * h - (gamma - 1)) / (gamma + 1)
    rho2 = rho * ((gamma + 1) * h) / ((gamma - 1) * h + 2)

    v2 = Ma2 * temp_to_sos(T2)
    v_x = v2 * np.cos(a)
    v_y = v2 * np.sin(a)
    return B, a, Ma2, T2, p2, rho2, v_x, v_y


def cone_shock(cone_angle, Ma, T, p, rho):
    """
    Computes properties of the conical oblique shock resulting
    from supersonic flow impinging on a cone in 3 dimensional flow.
    Inputs:
     - cone_angle is the half-angle of the 3D cone in radians.
     - Ma, T, p, and rho are the Mach number, temperature (K),
       pressure (Pa), and density (kg/m^3) of the flow.
    Returns:
     - shock angle in radians
     - flow redirection amount in radians
     - respectively, Mach number, temperature, pressure, density,
       and velocity components downstream of shock.
    Source: https://www.grc.nasa.gov/www/k-12/airplane/coneflow.html
    """

    wedge_angles = np.linspace(cone_angle, 0, 300)

    for wedge_angle in wedge_angles:
        B, a, Ma2, T2, p2, rho2, v_x, v_y = oblique_shock(wedge_angle, Ma, T, p, rho)
        v_theta = v_y * np.cos(B) - v_x * np.sin(B)
        v_r = v_y * np.sin(B) + v_x * np.cos(B)
        y0 = [v_r, v_theta]
        thetas = np.linspace(B, cone_angle, 2000)

        sol = odeint(taylor_maccoll, y0, thetas)
        if sol[-1, 1] < 0:
            return B, a, Ma2, T2, p2, rho2, v_x, v_y