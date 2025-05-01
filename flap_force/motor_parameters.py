import math
import numpy as np

def motor_parameters(motor_torque, motor_speed, gear_ratio, screw_lead, motor_voltage, motor_amps):
    """
    Calculates force and speed based on motor and mechanical parameters.

    Args:
        motor_torque (float): Motor torque (N·m)
        motor_speed (float): Motor speed (rpm)
        gear_ratio (float): Gearbox ratio (e.g., 1:X)
        screw_lead (float): Lead screw lead in mm/rev
        motor_voltage (float): Motor rated voltage (V)
        motor_amps (float): Motor rated current (A)

    Returns:
        tuple:
            - force (float): Output force in N
            - speed (float): Output speed in m/s
    """

    # Constants
    ARM_HORIZONTAL_IN = 1.5  # in inches
    ARM_HORIZONTAL_SI = ARM_HORIZONTAL_IN * 0.0254  # meters
    LEAD_SCREW_EFFICIENCY = 0.5  # range 0–1
    BATTERY_VOLTAGE = 3.7  # V per battery
    BATTERY_CAPACITY = 200  # mAh
    BATTERY_CAPACITY_SI = BATTERY_CAPACITY / 1000  # Ah

    # Apply gear ratio
    motor_torque *= gear_ratio
    motor_speed /= gear_ratio

    # Convert units
    motor_speed_si = motor_speed * 2 * math.pi / 60  # rad/s
    screw_lead_si = screw_lead / 1000  # m/rev

    # Motor power in Watts
    motor_wattage = motor_torque * motor_speed_si
    print(f"Motor Wattage: {motor_wattage:.2f} W")

    # Battery calculations
    voltage_batteries = motor_voltage / BATTERY_VOLTAGE
    number_of_batteries = math.ceil(voltage_batteries)
    print(f"Number of Batteries: {number_of_batteries}")

    battery_life = BATTERY_CAPACITY_SI / motor_amps
    print(f"Battery Life (hours): {battery_life:.2f}")

    # Output speed and force
    speed = (2 * ARM_HORIZONTAL_SI / screw_lead_si) / motor_speed_si
    force = motor_torque * 2 * math.pi * LEAD_SCREW_EFFICIENCY / screw_lead_si

    return force, speed
