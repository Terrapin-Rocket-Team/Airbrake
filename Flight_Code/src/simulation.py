from propagator import *
import serial
import time
from colorama import Fore
from ambiance import Atmosphere

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
                a_body = interial2Body(a, tilt_angle)
                line = (f"{baro1_atmosphere.pressure[0]/100},{baro1_atmosphere.T2t(baro1_atmosphere.temperature[0])[0]}," # Baro 1
                f"{gaussian_noise_generator(a_body[2], accel_error)},{gaussian_noise_generator(a_body[1], accel_error)},{gaussian_noise_generator(a_body[0], accel_error)}," # Accel
                f"{gaussian_noise_generator(0, gyro_error)},{gaussian_noise_generator(0, gyro_error)},{gaussian_noise_generator(0, gyro_error)}," # Gyro
                f"{0},{0},{0}," # Mag
                f"{0},{0},{r[2]},{0},") # GPS
        time.sleep(0.005)  # Slight delay to prevent CPU overload

ser.close()
