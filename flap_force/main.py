from motor_force_and_drag_force_2_max_flap import motor_force_and_drag_force_to_max_flap
from motor_parameters import motor_parameters
from drag_force import drag_force_from_mach
import numpy as np
import matplotlib.pyplot as plt

# Constants
force_drag_range = np.arange(1, 3001, 1)  # N
force_motor = motor_parameters(.144*2, 4000, 10, 2.54, 24, 1.67)[0]  # N

### Plot 1: Max Flap Angle vs Drag Force ###
# Define drag force range

# Compute angles for each drag force
angles = np.array([
    motor_force_and_drag_force_to_max_flap(force_motor, fd) for fd in force_drag_range
])

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(force_drag_range, angles, 'r-', linewidth=2)

# Add vertical lines
nominal_deploy_mach = .6
max_deploy_mach = .8
max_mach = 1.88
nominal_deploy_force = drag_force_from_mach(nominal_deploy_mach, 0.00645, .95, 6496)
max_deploy_force = drag_force_from_mach(max_deploy_mach, 0.00645, .95, 6096)
max_force = drag_force_from_mach(1.88, 0.00645, .95, 1524)
plt.axvline(x=nominal_deploy_force, color='purple', linestyle='--', linewidth=1.5, label=f'Nominal Deploy Force (Mach={nominal_deploy_mach})')
plt.axvline(x=max_deploy_force, color='blue', linestyle='--', linewidth=1.5, label=f'Max Deploy Force (Mach={max_deploy_mach})')
plt.axvline(x=max_force, color='green', linestyle='--', linewidth=1.5, label=f'Max Force (Mach = {max_mach})')

plt.xlabel('Drag Force (N)', fontsize=16)
plt.ylabel('Max Deployment Angle (deg)', fontsize=16)
plt.title(f'Max Deployment Angle vs Single Flap Drag Force (Motor Force = {force_motor:.0f} N)', fontsize=18)
plt.ylim([0, 90])
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid(True)
plt.tight_layout()
plt.legend(fontsize=14)
plt.savefig("output/max_flap_angle_vs_drag_force.png")
plt.close()
