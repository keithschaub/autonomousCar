import socket
import keyboard
import matplotlib.pyplot as plt
import numpy as np

# Replace with the IP address of your Arduino
arduino_ip = "192.168.86.42"
arduino_port = 80

# Global flag to control the running state
running = True

def on_exit(e):
    global running
    if e.name == 'esc':  # pressing the ESCAPE key closes the connection
        running = False

# Register the event listener
keyboard.on_press(on_exit)

# Radar plot setup
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)
ax.set_theta_zero_location('E')  # 0 degrees to the right
ax.set_ylim(0, 50)               # Distance range (0 to 50 cm)
data = []

def update_radar(line):
    global data
    if line:
        angle, distance = map(int, line.split(','))
        angle_rad = np.radians(angle)
        data.append((angle_rad, distance))
        if angle == 30 and len(data) > 1:  # Clear and restart when back to 45 degrees
            ax.clear()
            ax.set_ylim(0, 50)
            data = [(angle_rad, distance)]
        for angle_rad, distance in data:
            ax.plot(angle_rad, distance, 'ro', markersize=1)
            ax.plot([0, angle_rad], [0, distance], color='green', linewidth=0.2)
        plt.pause(0.1)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(0.5)  # Set a timeout for socket operations
    s.connect((arduino_ip, arduino_port))

    print("Connection Established. Press ESCAPE key to close connection and stop program")

    buffer = ""
    while running:
        try:
            received_data = s.recv(1024).decode('utf-8')
            if received_data:
                buffer += received_data
                while '\n' in buffer:
                    complete_message, buffer = buffer.split('\n', 1)
                    if complete_message.strip():
                        print(complete_message)
                        update_radar(complete_message)
        except socket.timeout:
            pass  # Ignore timeout errors
        plt.pause(0.1)  # Update the plot and yield control

    print("Closing connection")
    s.close()
    print("Connection closed")

keyboard.unhook_all()
