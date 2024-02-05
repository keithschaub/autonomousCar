import serial
import time

# Change this to the correct Bluetooth COM port
bluetooth_port = 'COM14'
baud_rate = 9600

# Initialize serial connection
ser = serial.Serial(bluetooth_port, baud_rate, timeout=1)

# Continuously read data
while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(f"Received: {line}")
        time.sleep(1)
