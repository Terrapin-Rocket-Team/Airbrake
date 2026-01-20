#!/usr/bin/env python3
"""
Test script to verify FlightDataSimulation can parse old flight data CSVs
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(__file__))

from desktop_simulation import FlightDataSimulation

def test_flight_data_csv():
    csv_file = "28_FlightData_with_pre.csv"

    if not os.path.exists(csv_file):
        print(f"ERROR: Could not find {csv_file}")
        return False

    print(f"Testing FlightDataSimulation with {csv_file}...")
    print("=" * 60)

    try:
        # Create simulation
        sim = FlightDataSimulation(csv_file, dt=0.02, ignition_delay=2.0)

        # Test a few steps
        print("\nTesting simulation steps:")
        for i in range(5):
            state = sim.step()
            sensor_data = sim.get_sensor_data(state)

            print(f"\nStep {i+1}:")
            print(f"  Time: {sensor_data['timestamp']:.3f}s")
            print(f"  Accel: [{sensor_data['accel'][0]:7.3f}, {sensor_data['accel'][1]:7.3f}, {sensor_data['accel'][2]:7.3f}] m/s²")
            print(f"  Gyro:  [{sensor_data['gyro'][0]:7.3f}, {sensor_data['gyro'][1]:7.3f}, {sensor_data['gyro'][2]:7.3f}] rad/s")
            print(f"  Mag:   [{sensor_data['mag'][0]:7.1f}, {sensor_data['mag'][1]:7.1f}, {sensor_data['mag'][2]:7.1f}] uT")
            print(f"  Pressure: {sensor_data['pressure']:.2f} hPa")
            print(f"  Temperature: {sensor_data['temperature']:.2f} °C")
            print(f"  Altitude: {state.position[2]:.2f} m")

        print("\n" + "=" * 60)
        print("SUCCESS: FlightDataSimulation is working correctly!")
        return True

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_flight_data_csv()
    sys.exit(0 if success else 1)
