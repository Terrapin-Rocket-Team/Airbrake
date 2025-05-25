from propagator import *
import serial
import time
from colorama import Fore
from ambiance import Atmosphere
from shock import cone_shock

# serialPort = '/dev/cu.usbmodem149805801'  # Use the correct port for the Arduino
serialPort = '/dev/cu.usbmodem166861901'
baudRate = 115200  # Match the baud rate to the Arduino's
# dataFile = '/Users/michaelmallamaci/Downloads/Jan_Airbrake_FlightData.csv'

# Open serial connection to Arduino
ser = serial.Serial(serialPort, baudRate)
time.sleep(2)  # Wait for the connection to establish
flapAngle = 0

if ser.is_open:
    line = "Sim Start"
    ser.write(b"telem/" + line.encode() + b'\n')
    print(f"{Fore.CYAN}" + 'S-' + line + Fore.RESET)

line = "DPS310 - Pres (hPa),DPS310 - Temp (C),\
BMI088andLIS3MDL - AccX,BMI088andLIS3MDL - AccY,BMI088andLIS3MDL - AccZ,BMI088andLIS3MDL - GyroX,BMI088andLIS3MDL - GyroY,BMI088andLIS3MDL - GyroZ,BMI088andLIS3MDL - MagX,BMI088andLIS3MDL - MagY,BMI088andLIS3MDL - MagZ,\
MAX-M10S - Lat,MAX-M10S - Lon,MAX-M10S - Alt (m),MAX-M10S - Fix Quality,\
BR - ALT (m),BR - PRES (Pa),BR - TEMP (C),BR - TILT (deg),BR - ROLL (deg),BR - VEL (m/s),BR - ACCX (m/s^2),BR - ACCY (m/s^2),BR - ACCZ (m/s^2),BR - GYROX (rad/s),BR - GYROY (rad/s),BR - GYROZ (rad/s)\
"
if ser.is_open:
    waitForResponse = False  # Flag to check if we are waiting for specific response
    while True:  # Keep checking until a correct response is received
        if not waitForResponse:
            print(f"{Fore.CYAN}" + 'S-' + line + Fore.RESET)
            ser.write(line.encode() + b'\n')
            waitForResponse = True  # Now wait for the response
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"{Fore.LIGHTGREEN_EX}" + 'R-' + response + Fore.RESET)
            if response.startswith("[][]"):
                waitForResponse = False  # Got the response, can send the next line
                flapAngle = float(response.split(",")[-1].strip())
                Propagate(flapAngle)
                baro1_atmosphere = Atmosphere(gaussian_noise_generator(r[2], baro_error)+ground_altitude)
                br_atmosphere = Atmosphere(gaussian_noise_generator(r[2], br_baro_error)+ground_altitude)
                a_body = interial2Body(a, tilt_angle)
                mach = np.linalg.norm(v)/atmosphere.speed_of_sound[0]
                print(mach)
                if mach > 1.25:
                    B, flow_redirection_amount, Ma2, T2, p2, rho2, v_x, v_y = cone_shock(np.deg2rad(10), mach, atmosphere.temperature[0], atmosphere.pressure[0], atmosphere.density[0])
                    baro1_pressure_pa = gaussian_noise_generator(p2, baro_error*10) # The *10 is a very rough approximation for meters to Pa (Pa/m ~= 12 @ sea level, Pa/m ~= 3 @ 10,000 m)
                    baro1_temperature_k = gaussian_noise_generator(T2, baro_error*.0065) # standard lapse rate is −6.5 K/km = -.0065 K/m in the troposphere (up to ~11,000 m)
                    br_pressure_pa = gaussian_noise_generator(p2, br_baro_error*10) # The *10 is a very rough approximation for meters to Pa (Pa/m ~= 12 @ sea level, Pa/m ~= 3 @ 10,000 m)
                    br_temperature_k = gaussian_noise_generator(T2, br_baro_error*.0065) # standard lapse rate is −6.5 K/km = -.0065 K/m in the troposphere (up to ~11,000 m)
                else:
                    baro1_pressure_pa = baro1_atmosphere.pressure[0]
                    baro1_temperature_k = baro1_atmosphere.temperature[0]
                    br_pressure_pa = br_atmosphere.pressure[0]
                    br_temperature_k = br_atmosphere.temperature[0]
                line = (f"{baro1_pressure_pa/100},{baro1_atmosphere.T2t(baro1_temperature_k)[0]}," # Baro 1
                f"{gaussian_noise_generator(a_body[2], accel_error)},{gaussian_noise_generator(a_body[1], accel_error)},{gaussian_noise_generator(a_body[0], accel_error)}," # Accel
                f"{gaussian_noise_generator(0, gyro_error)},{gaussian_noise_generator(0, gyro_error)},{gaussian_noise_generator(0, gyro_error)}," # Gyro
                f"{0},{0},{0}," # Mag
                f"{0},{0},{r[2]},{0}," # GPS
                f"{r[2]},{br_pressure_pa},{baro1_atmosphere.T2t(br_temperature_k)[0]},{0},{0},{np.linalg.norm(v)}," # Blue Raven Baro
                f"{gaussian_noise_generator(a_body[0], br_accel_error)},{gaussian_noise_generator(a_body[2], br_accel_error)},{gaussian_noise_generator(a_body[1], br_accel_error)}," # Blue Raven Accel
                f"{gaussian_noise_generator(0, br_gryo_error)},{gaussian_noise_generator(0, br_gryo_error)},{gaussian_noise_generator(0, br_gryo_error)}") # Blue Raven Gryo
        time.sleep(0.005)  # Slight delay to prevent CPU overload

ser.close()
