import socket
import keyboard
import matplotlib.pyplot as plt
import numpy as np
import threading
import queue
import time

# this program runs with UNO_R4_sweep_gyro_rpm (rpm is not yet integrated)
# the UNO_R4 sweeps and streams data, it also streams heading data whenever the car is moving forward, left, right, backward
# you move the car by using the UP/DOWN/LEFT/RIGHT arrow keys on the keyboard.

# Check your IP address!!!!

arduino_ip = "192.168.86.31"
arduino_port = 80
running = True
data_queue = queue.Queue()

# Open a file for writing
data_file = open("arduino_data.txt", "w")

# Radar plot setup
fig, ax = plt.subplots(subplot_kw={'polar': True})
ax.set_theta_zero_location('E')
ax.set_ylim(0, 80)
plt.ion()  # Turn on interactive mode for non-blocking plot updates

# Global variable for the heading text object
heading_text = None
heading_value = None
last_displayed_heading = None  # Global variable to track the last displayed heading


def on_key_event(e):
    global running, s
    try:
        if e.name == 'esc':
            running = False
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
        elif e.name == 'x':
            s.sendall('X'.encode('utf-8'))
            print('Sent X')

    except AttributeError:
        pass


def receive_data():
    global s, running, data_file  # Include data_file in global declaration
    s.settimeout(0.1)  # Set a short timeout for the recv operation
    while running:
        try:
            received_data = s.recv(1024).decode('utf-8')
            data_lines = received_data.strip().split('\n')
            for line in data_lines:
                # Write every line of received data to the file
                data_file.write(line + "\n")
                data_file.flush()

                if line.startswith("Heading"):
                    # Handle heading information
                    handle_heading(line)
                else:
                    # Handle sweep data
                    try:
                        angle, distance = map(int, line.split(','))
                        data_queue.put((angle, distance))
                    except ValueError:
                        pass  # Ignore lines that don't have angle, distance
        except socket.timeout:
            pass  # Ignore timeout errors


def handle_heading(heading_line):
    # Extract heading value from the line
    global heading_value
    heading_value = heading_line.split()[1]
    #print(f'Heading: {heading_value} deg')

def update_radar():
    global ax, running, heading_value, heading_text, last_displayed_heading
    count = 0 # counter for number of radar points plotted

    if heading_text is None:
        heading_text = ax.text(0.25, 1.1, '', fontsize=12, transform=ax.transAxes)

    while running:
        if not data_queue.empty():
            angle, distance = data_queue.get()
            angle_rad = np.radians(angle)

            #plot the data point
            ax.plot(angle_rad, distance, 'ro', markersize=1)
            ax.plot([0, angle_rad], [0, distance], color='green', linewidth=0.2)
            count +=1;  # increment the data counter to track the number of data points plotted

            # Update heading display if it has changed
            if heading_value is not None and heading_value != last_displayed_heading:
                if heading_text is not None:
                    heading_text.remove()  # Remove the old text object
                heading_text = ax.text(0.25, 1.1, f"Heading: {heading_value} deg", fontsize=12, transform=ax.transAxes)
                last_displayed_heading = heading_value

            # clear the plot after 100 data points ~4 sweeps
            if count >=50:
                ax.clear()
                ax.set_ylim(0, 80) # scale radar plot to max distance
                count = 0          # Reset counter
                s.sendall('X'.encode('utf-8'))  # send acknowledge plot is updated and ready to receive next packet

                # Reinitialize the heading text object after clearing the plot
                if heading_value is not None:
                    heading_text = ax.text(0.25, 1.1, f"Heading: {heading_value} deg", fontsize=12, transform=ax.transAxes)


            plt.draw()
            plt.pause(0.01)        # Short pause for plot update

        time.sleep(0.01)            # Short sleep to reduce CPU usage

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((arduino_ip, arduino_port))
    print("Connection Established. Press ESCAPE key to close connection and stop program")

    recv_thread = threading.Thread(target=receive_data)
    recv_thread.start()
    keyboard.on_press(on_key_event)

    update_radar()  # Call update_radar in the main thread

    recv_thread.join()
    print("Closing connection")
    keyboard.unhook_all()
    plt.close(fig)
    data_file.close()  # Close the file