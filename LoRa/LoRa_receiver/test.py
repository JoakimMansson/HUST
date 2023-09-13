import serial
import re
import win32com.client
import sys
import serial.tools.list_ports

def list_ports():
    ports = list(serial.tools.list_ports.comports())
    return [port.device for port in ports]

def check_data(port):
    try:
        ser = serial.Serial(port, baudrate=9600, timeout=1)
        data = ser.readline().decode().strip()
        if data:
            print("Data received from port", port)
            print("Data:", data)
            sys.exit()
    except Exception as e:
        print("Error: Could not open port", port)

ports = list_ports()
if not ports:
    print("No available USB ports found.")
    sys.exit()

for port in ports:
    check_data(port)

print("No data received from any of the available USB ports.")