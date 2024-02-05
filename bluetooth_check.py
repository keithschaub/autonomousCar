import serial
import time

import matplotlib.pyplot as plt
import numpy as np

simple_debug_flag = 0
radar_flag = 1
i = 0

if ( simple_debug_flag ):
    # Set up the serial connection (adjust COM port as needed)
    ser = serial.Serial('COM8', 9600, timeout=1)

    # if you have the correct COM port, it will immediately print values
    # if you have incorrect value, it will either timeout , or it won't print anything
    # if you reset power on arduino, you have to rerun this python script

    while True:
        try:
            data = ser.readline().decode('utf-8').strip().split(',')
            #print(data)
            #print(f"iteration: {i}")
            if len(data) == 2:
                angle, distance = map(int, data)
                print(f"iteration: {i}  Angle: {angle}, Distance: {distance} cm")
            i=i+1
        except ValueError:
            continue

if (radar_flag ):

    import matplotlib.pyplot as plt
    import numpy as np

    # Set up the serial connection (adjust COM port as needed)
    ser = serial.Serial('COM8', 9600, timeout=1)

    # Set up the matplotlib plot
    plt.ion()  # Interactive mode on
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_ylim(0, 100)  # Set distance range


    # Function to update the radar plot
    def update_radar(angle, distance):
        theta = np.radians(angle)
        color = 'green' if distance >= 30 else 'red'
        ax.plot([theta], [distance], marker='o', markersize=2, color=color)  # Plot the point
        plt.draw()
        plt.pause(0.01)


    # Reading and plotting loop
    while True:
        try:
            data = ser.readline().decode('utf-8').strip().split(',')
            if len(data) == 2:
                angle, distance = map(int, data)  # for integers
                # angle, distance = map(float, data) # Changed to float to handle decimal values
                if angle > 0:
                    update_radar(angle, distance) # add it to the radar map
                    print(f"iteration: {i}  Angle: {angle}, Distance: {distance} cm")
                if angle == -1:
                    print(f"                             avgDistance: {distance} cm")

                ser.write(b'A')  # Sending acknowledgement back to Arduino
                i += 1
        except Exception as e:  # Catching any exception and printing it
            print(f"Error: {e}")
            continue