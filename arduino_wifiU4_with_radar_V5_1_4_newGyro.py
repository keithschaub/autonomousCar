import socketio
import socket
import keyboard
import matplotlib.pyplot as plt
import numpy as np
import threading
import queue
import time

# Creat a Socket.IO Client to ship to Jake
sio = socketio.Client()

# Connect to the Flask-SocketIO Server
sio.connect('http://127.0.0.1:5000')

# Global variables for position text objects
pos_text = None
posX_value = 0
posY_value = 0

arduino_ip = "192.168.86.20"
arduino_port = 80
running = True
data_queue = queue.Queue()

# Open a file for writing
data_file = open("arduino_data_V5_1_4newGyro_LeftRight.txt", "w")

# Grid plot setup
fig, (ax, ax2) = plt.subplots(2, 1)
ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.grid(True)

# Setup for gyroZ plot
ax2.set_xlim(0, 60)  # Assuming you have 61 lines and want to plot each as a point
ax2.set_ylim(-2, 2)  # Adjust based on expected gZ values
ax2.set_title('gZ values over Time')
ax2.set_xlabel('Time (arbitrary units)')
ax2.set_ylabel('gZ')

plt.ion()  # Turn on interactive mode for non-blocking plot updates

# Plot the initial position of the car
car_marker, = ax.plot(0, 0, 'bo', markersize=5, label='Car')

# Adjust the initial setup for heading_text and add pos_text
heading_value = 90
heading_text = ax.text(-190, 220, '', fontsize=10)  # Adjust y value as needed to display above the grid
pos_text = ax.text(-190, 200, '', fontsize=10)  # New text object for displaying posX and posY


def on_key_event(e):
    global running, s
    try:
        if e.name == 'esc':
            running = False
            sio.disconnect() # disconnect from the Socket.IO server when stopping
        elif e.name == 'a':
            s.sendall('A'.encode('utf-8'))
            print('Sent A')
        elif e.name == 'up':
            s.sendall('F'.encode('utf-8'))
            print('Sent F')
        elif e.name == 'down':
            s.sendall('B'.encode('utf-8'))
            print('Sent B')
        elif e.name == 'left':
            s.sendall('L'.encode('utf-8'))
            print('Sent L')
        elif e.name == 'right':
            s.sendall('R'.encode('utf-8'))
            print('Sent R')
        elif e.name == 'c':
            s.sendall('C'.encode('utf-8'))
            print('Sent C')
        elif e.name == 'x':
            s.sendall('X'.encode('utf-8'))
            print('Sent X')

    except AttributeError:
        pass


def receive_data():
    global s, running, posX_value, posY_value
    gZ_values = []
    time_values= []
    s.settimeout(0.1)  # Set a short timeout for the recv operation
    while running:
        try:
            received_data = s.recv(8192).decode('utf-8')
            data_lines = received_data.strip().split('\n')
            for line in data_lines:
                data_file.write(line + "\n")  # Write every line of received data to the file
                if "posX" in line or "posY" in line:
                    update_position(line)
                elif line.startswith("Heading"):
                    update_heading(line)
                elif "gZ:" in line:
                    parts = line.split(', ')
                    for part in parts:
                        if "gZ:" in part:
                            gZ_value = float(part.split(':')[1])
                            gZ_values.append(gZ_value)
#                        if "timeS:" in part:
#                            time_value = float(part.split(':')[1]) / 1000
#                            time_values.append(time_value)
                else:
                    try:
                        angle, distance = map(int, line.split(','))
                        # Convert polar to Cartesian coordinates and plot
                        sio.emit('newdata', {'angle': angle, 'distance': distance})
                        #x, y = polar_to_cartesian(angle, distance)
                        x, y = polar_to_cartesian_new(angle, distance)
                        if (distance >0 and distance < 70):
                            ax.plot(x, y, 'ro', markersize=1)
                    except ValueError:
                        pass  # Ignore lines that don't have angle, distance
            if gZ_values:
                ax2.clear()
                time_values = np.arange(0, len(gZ_values))
                ax2.plot(time_values, gZ_values, 'ro', markersize = 1)
                gZ_values = []
                time_values = []
            time.sleep(1.5)
            s.sendall('X'.encode('utf-8'))
        except socket.timeout:
            pass  # Ignore timeout errors

def polar_to_cartesian(angle, distance):
    # Convert polar coordinates (angle, distance) to Cartesian (x, y)
    radians = np.radians(angle)
    x = distance * np.cos(radians)
    y = distance * np.sin(radians)
    return x, y

def polar_to_cartesian_new(angle, distance):
    global heading_value
    # Convert ultrasound (angle, distance) to world map Cartesian (x, y)
    ultrasound_offset = 8.4 # cm physically measured distance from car's central point to the servo
    heading_radians = np.radians(heading_value)

    ultrasound_x = posX_value + ultrasound_offset * np.cos(heading_radians)
    ultrasound_y = posY_value + ultrasound_offset * np.sin(heading_radians)

    theta_total_deg = heading_value + (angle - 90)
    theta_total_rad = theta_total_deg * np.pi / 180

    x = round( ultrasound_x + distance * np.cos(theta_total_rad) , 1)
    y = round( ultrasound_y + distance * np.sin(theta_total_rad) , 1)

    print(f'angle {angle}, distance {distance}, x {x}, y {y}')

    return x, y

def update_position(line):
    global posX_value, posY_value, car_marker, pos_text
    parts = line.split(',')
    for part in parts:
        if "posX" in part:
            posX_value = float(part.split()[1])
        elif "posY" in part:
            posY_value = float(part.split()[1])
    car_marker.set_data(posX_value, posY_value)
    pos_text.set_text(f"PosX: {posX_value}, PosY: {posY_value}")  # Update pos_text with new values

    sio.emit('pos_data', {'x': posX_value, 'y': posY_value})



def update_heading(line):
    global heading_text, heading_value
    parts = line.split()
    if len(parts) >=2:
        try:
            heading_str = parts[1].replace(',', '')
            heading_value = float(heading_str)
            heading_text.set_text(f"Heading: {heading_value} deg")
            sio.emit('heading_data', {'heading': heading_value})
        except ValueError as e:
            print(f"Error converting heading to float: {e}. Received line: '{line}'")
    else:
        print(f"Received line does not have enough parts for heading: {line}'")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((arduino_ip, arduino_port))
    print("Connection Established. Press ESCAPE key to close connection and stop program")

    recv_thread = threading.Thread(target=receive_data)
    recv_thread.start()
    keyboard.on_press(on_key_event)

    while running:
        plt.draw()
        plt.pause(0.01)  # Short pause for plot update
        #time.sleep((0.01)

    recv_thread.join()
    print("Closing connection")
    keyboard.unhook_all()
    plt.close(fig)
    data_file.close()  # Close the file
