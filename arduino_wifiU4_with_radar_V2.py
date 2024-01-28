import socket
import keyboard
import matplotlib.pyplot as plt
import numpy as np
import threading
import queue
import time

arduino_ip = "192.168.86.42"
arduino_port = 80
running = True
data_queue = queue.Queue()

# Radar plot setup
fig, ax = plt.subplots(subplot_kw={'polar': True})
ax.set_theta_zero_location('E')
ax.set_ylim(0, 80)
plt.ion()  # Turn on interactive mode for non-blocking plot updates

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
    global s, running
    s.settimeout(0.1)  # Set a short timeout for the recv operation
    while running:
        try:
            received_data = s.recv(1024).decode('utf-8')
            print(received_data)
            sweeps = received_data.strip().split('\n')
            for sweep in sweeps:
                angle, distance = map(int, sweep.split(','))
                data_queue.put((angle, distance))
        except socket.timeout:
            pass  # Ignore timeout errors

def update_radar():
    global ax, running
    count = 0 # counter for number of radar points plotted
    while running:
        if not data_queue.empty():
            angle, distance = data_queue.get()
            angle_rad = np.radians(angle)

            #plot the data point
            ax.plot(angle_rad, distance, 'ro', markersize=1)
            ax.plot([0, angle_rad], [0, distance], color='green', linewidth=0.2)
            count +=1;  # increment the data counter to track the number of data points plotted

            # clear the plot after 100 data points ~4 sweeps
            if count >=50:
                ax.clear()
                ax.set_ylim(0, 80) # scale radar plot to max distance
                count = 0          # Reset counter
                s.sendall('X'.encode('utf-8'))  # send acknowledge plot is updated and ready to receive next packet

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
