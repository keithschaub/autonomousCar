import serial
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow # Import Arrow
import numpy as np
from sklearn.cluster import DBSCAN
import time

# Parameters
grid_size = 400 # 600  # 200
center = grid_size // 2
arrow_angle = 90  # Initial arrow angle
i = 0


# Initialize the grid
points = np.zeros((grid_size, grid_size))
collected_points = []

# New global variable for plotting
plot_points = np.zeros_like(points)

# Set up the serial connection (adjust COM port as needed)
ser = serial.Serial('COM8', 9600, timeout=1)

# Initialize plot outside the loop
if (0):
    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_xlim(-grid_size/2, grid_size/2)
    ax.set_ylim(-grid_size/2, grid_size/2)

# Function to add points to the collected list
def collect_data(angle, distance):
    global collected_points, center
    # Convert polar to cartesian and add to the list
    if angle <= 90:
        # For angles 0-90 degrees
        adjusted_angle = np.radians(angle)
        x = int(center + distance * np.cos(adjusted_angle))
        y = int(center - distance * np.sin(adjusted_angle))
    else:
        # For angles 91-180 degrees
        adjusted_angle = np.radians(angle - 90)
        x = int(center - distance * np.sin(adjusted_angle))
        y = int(center - distance * np.cos(adjusted_angle))

    collected_points.append((x, y))


# Function to filter points using DBSCAN
# Initialize the last time the filter was run
last_filter_time = time.time()

eps = 8  # 20 #5         # Maximum distance between two points (to be tuned)
min_samples = 5 # 10  # Min # of points to form a dense region (to be tuned)

def filter_points():
    global collected_points, plot_points
    if collected_points:
        # Convert to numpy array for DBSCAN
        data = np.array(collected_points)

        # Run DBSCAN clustering
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(data)

        # Labels for each point in the dataset. Noise points are labeled as -1
        labels = db.labels_

        # Filter out noise points
        valid_points = data[labels != -1]

        # Clear the points and update with valid points
        #points = np.zeros((grid_size, grid_size))
        #plot_points = np.zeros_like(points)
        for x, y in valid_points:
            if 0 <= x < grid_size and 0 <= y < grid_size:
                plot_points[int(y), int(x)] = 1  # Set the point on the grid

        # Clear the collected points list
        collected_points = []


# code not being used
last_valid_readings = []
N = 5
def is_valid_point(new_reading, last_readings, max_deviation=25, max_angle_change=30):
    """
    Checks if the new reading is a valid point based on past readings.
    max_deviation: maximum allowed distance deviation from the running average.
    max_angle_change: maximum allowed angle change between consecutive readings.
    """
    # Calculate running average distance and angle
    if last_readings:
        avg_distance = np.mean([reading['distance'] for reading in last_readings])
        avg_angle = np.mean([reading['angle'] for reading in last_readings])

        # Calculate deviation from average
        distance_deviation = abs(new_reading['distance'] - avg_distance)
        angle_deviation = abs(new_reading['angle'] - avg_angle)

        # Debug prints to help you understand why readings are being rejected
        print(f"Average distance: {avg_distance}, New distance: {new_reading['distance']}, Deviation: {distance_deviation}")
        print(f"Average angle: {avg_angle}, New angle: {new_reading['angle']}, Deviation: {angle_deviation}")

        return distance_deviation <= max_deviation and angle_deviation <= max_angle_change
    return False


# Event handler for key presses
def on_key(event):
    global ser  # Make sure to use the global serial connection
    if event.key == 'up':
        shift_points('up')
        print('moving forward')
        ser.write(b'F')  # Send 'F' to move forward
    elif event.key == 'down':
        print('moving backward')
        #shift_points('down')
        #ser.write(b'B')  # Send 'B' to move backward
    elif event.key == 'left':
        print('turning left')
        shift_points('left')
        ser.write(b'L')   # Send 'L' to rotate left

    elif event.key == 'right':  #limiting the car to only turn left
        print('turning right')
        shift_points('right')
        #ser.write(b'R')    # Send 'R' to rotate right

# Initial Setup
plt.figure(figsize=(8, 8))
ax = plt.gca()
ax.set_xlim(-grid_size/2, grid_size/2)
ax.set_ylim(-grid_size/2, grid_size/2)
ax.grid(which='both')
ax.set_xticks(range(-grid_size//2, grid_size//2 + 1, 10))
ax.set_yticks(range(-grid_size//2, grid_size//2 + 1, 10))

# Static elements
ax.scatter(0, 0, color='blue', s=50)  # Blue point at the center with smaller size
ax.add_patch(Arrow(0, 0, 0, 10, width=10, color='blue'))  # Arrow indicating direction

plt.gcf().canvas.mpl_connect('key_press_event', on_key)
plt.draw()

scatter_plot = ax.scatter([], [], color = 'red', s=6)

def update_plot():
    global scatter_plot
    y_coords, x_coords = np.where(plot_points == 1)
    scatter_plot.set_offsets(np.c_[x_coords - center, center - y_coords])

def draw_grid(points, arrow_angle):
    """Draw the grid with the points and the arrow."""
#    plt.imshow(points, cmap='Reds', extent=(-grid_size/2, grid_size/2, -grid_size/2, grid_size/2))
#    plt.grid(which='both')
#    plt.xticks(range(-grid_size//2, grid_size//2 + 1, 10))
#    plt.yticks(range(-grid_size//2, grid_size//2 + 1, 10))

    # Draw the points on the grid
    y_coords, x_coords = np.where(points == 1)
    plt.scatter(x_coords - center, center - y_coords, color ='red', s=3) # s is the size of the point

    # Draw the blue point at the center (0,0) that never moves
#    plt.scatter(0, 0, color='blue', s=100)  # s is the size of the point

    # Calculate arrow direction based on the current angle and make it bigger
    #dx = 20 * np.cos(np.radians(arrow_angle))  # Adjusted for angle interpretation
    #dy = 20 * np.sin(np.radians(arrow_angle))  # Adjusted for angle interpretation
    #plt.gca().add_patch(Arrow(0, 0, dx, dy, width=10, color='blue'))  # Increased arrow size
#    dx = 0
#    dy = 10
#    plt.gca().add_patch(Arrow(0, 0, dx, dy, width=10, color='blue'))  # Increased arrow size

#    plt.draw()

def update_grid_with_bluetooth_data(angle, distance):
    """Update grid points based on Bluetooth data."""
    global plot_points

    if angle <= 90:
        # For angles 0-90 degrees
        adjusted_angle = np.radians(angle)
        x = int(center + distance * np.cos(adjusted_angle))
        y = int(center - distance * np.sin(adjusted_angle))
    else:
        # For angles 91-180 degrees
        adjusted_angle = np.radians(angle - 90)
        x = int(center - distance * np.sin(adjusted_angle))
        y = int(center - distance * np.cos(adjusted_angle))

    # Create a dictionary for the new reading with Cartesian coordinates
    new_reading = {'x': x, 'y': y, 'distance': distance, 'angle': angle}

    if 0 <= x < grid_size and 0 <= y < grid_size:
        filter = False
        # check if the new reading is valid before plotting
        if filter:
            # Initialize the last_valid_readings with the first few readings
            if not last_valid_readings:
                last_valid_readings.append(new_reading)
            elif is_valid_point(new_reading, last_valid_readings, max_deviation=25, max_angle_change = 30):
                points[y, x] = 1 # Set the point on the grid
                # update the list of last valid readings
                last_valid_readings.append(new_reading)
                if len(last_valid_readings) > N:
                    last_valid_readings.pop(0)
        else:
            plot_points[y, x] = 1                 # for now, place the point on the map as valid
            #if distance < 70:     # if the distance > 50, don't put it on the grid
            #    points[y, x] = 1  # Set the point on the grid

def rotate_points(angle, sensor_offset_front=5, wheelbase=7.9):
    """Rotate points around the center of rotation considering the sensor's front offset."""
    global plot_points, center

    # The center of rotation is halfway between the front and rear axles
    center_of_rotation_y = center - (wheelbase / 2.0) + sensor_offset_front

    angle_rad = np.radians(angle)  # Convert angle to radians
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    new_points = np.zeros_like(plot_points)

    for y in range(grid_size):
        for x in range(grid_size):
            if plot_points[y, x] == 1:
                # Translate point to the rotation axis
                trans_x = x - center
                trans_y = y - center_of_rotation_y

                # Rotate the point
                rotated_x = trans_x * cos_angle - trans_y * sin_angle
                rotated_y = trans_x * sin_angle + trans_y * cos_angle

                # Translate the point back
                new_x = int(rotated_x + center)
                new_y = int(rotated_y + center_of_rotation_y)

                if 0 <= new_x < grid_size and 0 <= new_y < grid_size:
                    new_points[new_y, new_x] = 1

    plot_points = new_points


'''def rotate_points(angle, sensor_offset = 8.95):
    """Rotate points around the center."""
    # The distance between front and rear axles is 7.9 cm
    # the midpoint is the center of rotation, let's call that (0, 0)
    # the sensor is mounted 5 cm in front of the front axle
    # thus the sensor's distance from the center of rotation is 5 + 7.9/2 = 8.95cm
    # as the car rotates (turns left) in a circle,the sensor travels along a circle of radius 8.95cm

    global plot_points, center
    angle_rad = np.radians(angle)  # positive for counter clockwise rotation
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    #new_points = np.zeros((grid_size, grid_size))
    new_points = np.zeros_like(plot_points)

    # translate points to account for sensor offset
    dx_offset = sensor_offset * np.cos(angle_rad)
    dy_offset = sensor_offset * np.sin(angle_rad)

    for y in range(grid_size):
        for x in range(grid_size):
            if plot_points[y, x] == 1:
                # Translate point to origin
                trans_x = (x - center) - dx_offset
                trans_y = (y - center) - dy_offset

                # Rotate point
                rotated_x = trans_x * cos_angle - trans_y * sin_angle
                rotated_y = trans_x * sin_angle + trans_y * cos_angle

                # Translate point back
                new_x = int(rotated_x + center + dx_offset)
                new_y = int(rotated_y + center + dy_offset)

                # Check if new position is within bounds
                if 0 <= new_x < grid_size and 0 <= new_y < grid_size:
                    new_points[new_y, new_x] = 1

    plot_points = new_points
'''

def shift_points(direction):
    """Shift points based on the arrow key pressed and arrow direction."""
    global plot_points, arrow_angle
    # Set travel distance based on direction
    rotation = 12.85  # LEFT for 106ms ~=12.85 degrees
    #rotation = 90  # LEFT for 106ms ~=12.85 degrees
    travel_distance = 23 if direction == 'up' else -23

    # Calculate shift direction based on arrow orientation
    dy = np.round(travel_distance * np.sin(np.radians(arrow_angle)))
    dx = np.round(travel_distance * np.cos(np.radians(arrow_angle)))

    if direction in ['up', 'down']:
        # Shift points based on calculated dx and dy
        new_points = np.zeros((grid_size, grid_size))
        for y in range(grid_size):
            for x in range(grid_size):
                new_x = x + int(-dx)
                new_y = y + int(dy)
                if 0 <= new_x < grid_size and 0 <= new_y < grid_size:
                    new_points[new_y, new_x] = plot_points[y, x]
        plot_points = new_points

    elif direction == 'left':
        rotate_points(rotation)
        arrow_angle = (arrow_angle + rotation) % 90  # Rotate arrow by +12.85 degrees

    elif direction == 'right':
        rotate_points(-rotation)
        arrow_angle = (arrow_angle - rotation) % 90  # Rotate arrow by -12.85 degrees

    #plt.clf()
    #draw_grid(points, arrow_angle)
    update_plot()

    print(f'arrow_angle = {arrow_angle}')
    print(f'dx = {dx}')
    print(f'dy = {dy}')
    print()

dbscan_filter_flag = False
# Main loop
while True:
    try:
        data = ser.readline().decode('utf-8').strip().split(',')
        #print(data)
        if len(data) == 2:
            angle, distance = map(int, data)

            # angle, distance = map(float, data) # Changed to float to handle decimal values
            if angle >= 0:
                if (dbscan_filter_flag):
                    collect_data(angle, distance)
                    print(f'collected: angle: {angle}  deg, distance: {distance}  cm')
                else:
                    update_grid_with_bluetooth_data(angle, distance)
                    print(f"iteration: {i}  Angle: {angle}, Distance: {distance} cm")
            if angle == -1:
                print(f"                             avgDistance: {distance} cm")

            ser.write(b'A')  # Sending acknowledgement back to Arduino
            i += 1

        if (dbscan_filter_flag):

            # periodically filter the points
            current_time = time.time()
            if current_time - last_filter_time >= 3:
                filter_points()
                last_filter_time = current_time

                # update plot with filtered points
                #plt.clf()
                #y_coords, x_coords = np.where(points == 1)
                #ax.scatter(x_coords - center, center - y_coords, color='red', s=6)  # Slightly larger red points
                update_plot()
                plt.draw()
                plt.pause(0.01)

        else:
            #draw_grid(points, arrow_angle)
            update_plot()
            plt.draw()
            plt.pause(0.01)

    except Exception as e:
        print(f"Error: {e}")
        continue
