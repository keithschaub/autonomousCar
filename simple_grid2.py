import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arrow

# Parameters
grid_size = 200
plot_points = np.zeros((grid_size, grid_size))
# Setting initial points near the center (considering 0,0 as the center)
center = grid_size // 2
plot_points[center - 35, center - 10:center + 10] = 1
plot_points[center - 25, center - 10:center + 10] = 1

plot_points[center - 34:center - 25, center - 10] = 1
plot_points[center - 34:center - 25, center + 10] = 1


# Triangle
plot_points[center + 20, center + 10:center + 15] = 1
plot_points[center + 21, center + 11:center + 14] = 1
plot_points[center + 22, center + 12:center + 13] = 1

arrow_angle = 90  # Initial arrow angle

# Function definitions
def draw_grid(points, arrow_angle):
    """Draw the grid with the points and the arrow."""
    # Set the figure size
    #plt.figure(figsize=(8, 8))

    # Display the grid
    plt.imshow(points, cmap='Reds', extent=(-grid_size / 2, grid_size / 2, -grid_size / 2, grid_size / 2))
    plt.grid(which='both')
    plt.xticks(range(-grid_size // 2, grid_size // 2 + 1, 5))
    plt.yticks(range(-grid_size // 2, grid_size // 2 + 1, 5))

    # Find the points to plot
    y_coords, x_coords = np.where(points == 1)

    # Scatter plot the points in red and with a larger size
    plt.scatter(x_coords - center, center - y_coords, color='red', s=1)

    # Draw the arrow
    dx = 5 * np.sin(np.radians(arrow_angle))
    dy = 5 * np.cos(np.radians(arrow_angle))
    plt.gca().add_patch(Arrow(0, 0, dx, dy, width=3, color='blue'))

    # Refresh the plot
    plt.draw()


import numpy as np

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

def shift_points(direction):
    global plot_points, arrow_angle
    """Shift points based on the arrow key pressed and arrow direction."""
    global plot_points, arrow_angle
    # Set travel distance based on direction
    #rotation = 12.85  # LEFT for 106ms ~=12.85 degrees
    rotation = 90  # LEFT for 106ms ~=12.85 degrees
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
        arrow_angle = (arrow_angle + rotation) % 360  # Rotate arrow by +12.85 degrees

    elif direction == 'right':
        rotate_points(-rotation)
        arrow_angle = (arrow_angle - rotation) % 360  # Rotate arrow by -12.85 degrees

    #plt.clf()
    #draw_grid(points, arrow_angle)
  #  update_plot()

    print(f'arrow_angle = {arrow_angle}')
    print(f'dx = {dx}')
    print(f'dy = {dy}')
    print()

def on_key(event):
    if event.key in ['up', 'down', 'left', 'right']:
        shift_points(event.key)
        plt.clf()  # Clear the current figure
        draw_grid(plot_points, arrow_angle)  # Redraw the grid with the updated points and arrow angle
        plt.pause(0.01)  # Pause to update the figure

# Set up plot
plt.figure(figsize=(8, 8))
draw_grid(plot_points, arrow_angle)
plt.gcf().canvas.mpl_connect('key_press_event', on_key)

# Display the plot
plt.show()
