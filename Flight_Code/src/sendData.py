import serial
import time
from colorama import Fore

serialPort = '/dev/cu.usbmodem157708301'  # Use the correct port for the Arduino
baudRate = 115200  # Match the baud rate to the Arduino's
dataFile = '/Users/michaelmallamaci/Downloads/Jan_Airbrake_FlightData.csv'
#output = '/Users/michaelmallamaci/Downloads/test_results.csv'

# Open serial connection to Arduino
ser = serial.Serial(serialPort, baudRate)
time.sleep(2)  # Wait for the connection to establish

if ser.is_open:
    with open(dataFile, 'r') as file: # open(output, 'w', newline='') as outCsv:
        #outCsv.write("timeAbsolute, timeSinceLaunch,stage,timeSincePreviousStage,AX,AY,AZ,VX,VY,VZ,PX,PY,PZ,OX,OY,OZ,OW,APOGEE\n")
        waitForResponse = False  # Flag to check if we are waiting for specific response
        for line in file:
            line = line.strip()
            if line:
                while True:  # Keep checking until a correct response is received
                    if not waitForResponse:
                        print(f"{Fore.CYAN}" + 'S-' + line + Fore.RESET)
                        ser.write(line.encode() + b'\n')
                        waitForResponse = True  # Now wait for the response
                    if ser.in_waiting > 0:
                        response = ser.readline().decode().strip()
                        print(f"{Fore.LIGHTGREEN_EX}" + 'R-' + response + Fore.RESET)
                        if response.startswith("[][]"):
                            #outCsv.write(response.replace("|", ",") + '\n')
                            waitForResponse = False  # Got the response, can send the next line
                            break  # Exit the while loop to proceed with next line
                    time.sleep(0.005)  # Slight delay to prevent CPU overload

ser.close()