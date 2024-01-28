import serial
import matplotlib.pyplot as plt
import threading

# Parameters
i = 0

# Set up the serial connection (adjust COM port as needed)
ser = serial.Serial('COM8', 9600, timeout=1)

# Event handler for key presses
def on_key(event):
    global ser  # Make sure to use the global serial connection
    if event.key == 'up':
        print('moving forward')
        ser.write(b'F')  # Send 'F' to move forward
    elif event.key == 'down':
        print('moving backward')
        ser.write(b'B')  # Send 'B' to move backward
    elif event.key == 'left':
        print('turning left')
        ser.write(b'L')   # Send 'L' to rotate left
    elif event.key == 'right':
        print('turning right')
        ser.write(b'R')    # Send 'R' to rotate right

# Function to read from the serial port
def read_serial():
    global ser, i
    while True:
        try:
            data = ser.readline().decode('utf-8').strip().split(',')
            print(data)
            if len(data) == 2:
                angle, distance = map(int, data)
                i += 1
        except Exception as e:
            print(f"Error: {e}")

# Create a separate thread for reading from the serial port
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True  # This ensures that the thread will close when the main program closes
serial_thread.start()

# Create a dummy plot to capture key events
plt.figure(figsize=(5, 3))
plt.title("Press arrow keys to control")
plt.plot([0, 1], [0, 1])  # Dummy plot

# Connect the event handler
plt.gcf().canvas.mpl_connect('key_press_event', on_key)

plt.show()
