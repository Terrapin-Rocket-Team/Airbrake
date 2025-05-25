import math
import numpy as np

def calculate_friction_coefficient(reynolds: float, mach: float, surface_roughness: float) -> float:
    """Calculate skin friction coefficient"""
    if not surface_roughness:
        # Perfect finish (partially laminar)
        if reynolds < 1.0e4:
            cf = 1.33e-2
        elif reynolds < 5.39e5:
            cf = 1.328 / math.sqrt(reynolds)
        else:
            cf = 1.0 / pow(1.50 * math.log(reynolds) - 5.6, 2) - 1700/reynolds
    else:
        # Turbulent
        if reynolds < 1.0e4:
            cf = 1.48e-2
        else:
            cf = 1.0 / pow(1.50 * math.log(reynolds) - 5.6, 2)

    # Compressibility corrections
    if mach < 0.9:
        cf *= 1 - 0.1 * mach**2
    elif mach > 1.1:
        if not surface_roughness:
            cf *= 1.0 / pow(1 + 0.045 * mach**2, 0.25)
        else:
            cf *= 1.0 / pow(1 + 0.15 * mach**2, 0.58)
    else:
        # Linear interpolation between M=0.9 and M=1.1
        if not surface_roughness:
            c1 = 1 - 0.1 * 0.9**2
            c2 = 1.0 / pow(1 + 0.045 * 1.1**2, 0.25)
        else:
            c1 = 1 - 0.1 * 0.9**2
            c2 = 1.0 / pow(1 + 0.15 * 1.1**2, 0.58)
        cf *= c2 * (mach - 0.9) / 0.2 + c1 * (1.1 - mach) / 0.2
        
    return cf

def calculate_stagnation_cd(mach: float) -> float:
    """Calculate stagnation pressure coefficient"""
    if mach <= 1:
        pressure = 1 + mach**2/4 + mach**4/40
    else:
        pressure = 1.84 - 0.76/mach**2 + 0.166/mach**4 + 0.035/mach**6
    return 0.85 * pressure

def calculate_base_cd(mach: float) -> float:
    """Calculate base drag coefficient"""
    if mach <= 1:
        return 0.12 + 0.13 * mach**2
    else:
        return 0.25 / mach

def calculate_roughness_correction(mach: float, surface_roughness: float, rocket_length: float) -> float:
    """Calculate roughness effects"""
    if not surface_roughness:
        return 0
    
    if mach < 0.9:
        correction = 1 - 0.1 * mach**2
    elif mach > 1.1:
        correction = 1 / (1 + 0.18 * mach**2)
    else:
        c1 = 1 - 0.1 * 0.9**2
        c2 = 1 / (1 + 0.18 * 1.1**2)
        correction = c2 * (mach - 0.9) / 0.2 + c1 * (1.1 - mach) / 0.2
        
    return 0.032 * pow(surface_roughness/rocket_length, 0.2) * correction

# Used OpenRocket's Coefficient of Drag Approach
def total_drag_coefficient(reynolds, mach_number: float, surface_roughness: float, rocket_length: float) -> float:
    """
    Calculate the total drag coefficient by summing wave drag, base drag, and friction drag.
    :param reynolds: Reynolds Number
    :param mach_number: Mach number (ratio of rocket velocity to speed of sound)
    """
    
    # Calculate friction coefficient
    cf = calculate_friction_coefficient(reynolds, mach_number, surface_roughness)
    
    # Calculate roughness effects
    roughness_cd = calculate_roughness_correction(mach_number, surface_roughness, rocket_length)
    
    # Use maximum of Cf and roughness-limited value
    friction_cd = max(cf, roughness_cd)
    
    # Calculate pressure drag
    stagnation_cd = calculate_stagnation_cd(mach_number)/2
    
    # Calculate base drag
    base_cd = calculate_base_cd(mach_number)
    
    total_cd = friction_cd + stagnation_cd + base_cd

    print(f"{'Friction Cd':>12} | {'Stagnation Cd':>14} | {'Base Cd':>8} | {'Total Cd':>9}")
    print("-" * 55)
    print(f"{friction_cd:12.5f} | {stagnation_cd:14.5f} | {base_cd:8.5f} | {total_cd:9.5f}")
    return total_cd
