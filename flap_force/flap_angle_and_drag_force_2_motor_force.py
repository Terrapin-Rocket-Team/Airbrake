import math

def flap_angle_and_drag_force_to_motor_force(theta_deg, force_drag):
    """
    Calculates the motor force and pushrod force based on flap angle and drag force
    in the 2024/25 airbrake design.

    Args:
        theta_deg (float): Flap angle in degrees (0 = closed, 90 = fully deployed)
        force_drag (float): Drag force on one flap (any unit of force)

    Returns:
        tuple: (force_motor, force_pushrod)
            - force_motor: Total motor force for 4 flaps (same unit as force_drag)
            - force_pushrod: Compression force in one pushrod (same unit)
    """

    # Geometric constants in inches
    length_flap = 4.5  # inches
    length_pushrod = 4.3  # inches
    length_central_pushrod_connection_to_hinge = 2  # inches
    length_clevis = 0.61  # inches

    # Convert angle to radians
    theta_rad = math.radians(theta_deg)

    # Force perpendicular to flap due to drag
    force_perpendicular = force_drag * math.sin(theta_rad)

    # Angle psi (between clevis and flap)
    psi = math.radians(90 - theta_deg) + math.atan2(2 * length_clevis, length_flap)

    # Intermediate length from geometry
    intermediate = (length_flap / 2)**2 + length_clevis**2
    term = length_central_pushrod_connection_to_hinge + math.sqrt(intermediate) * math.cos(psi)

    # Angle phi (between pushrod and connection)
    phi = math.acos(term / length_pushrod)

    # Pushrod and motor forces
    force_pushrod = -force_perpendicular / math.sin(phi - psi)
    force_motor = 4 * force_pushrod * math.sin(phi)

    return force_motor, force_pushrod
