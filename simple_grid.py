import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arrow

# Parameters
grid_size = 200
points = np.zeros((grid_size, grid_size))
# Setting initial points near the center (considering 0,0 as the center)
center = grid_size // 2
points[center - 71, center + 3:center + 8] = 1
points[center - 70, center + 3:center + 8] = 1

# Triangle
points[center + 20, center + 10:center + 15] = 1
points[center + 21, center + 11:center + 14] = 1
points[center + 22, center + 12:center + 13] = 1

arrow_angle = 0  # Initial arrow angle



def draw_grid(points, arrow_angle):
    """Draw the grid with the points and the arrow."""
    plt.imshow(points, cmap='Reds', extent=(-grid_size/2, grid_size/2, -grid_size/2, grid_size/2))
    plt.grid(which='both')
    plt.xticks(range(-grid_size//2, grid_size//2 + 1, 5))
    plt.yticks(range(-grid_size//2, grid_size//2 + 1, 5))

    # Draw the blue point at the center (0,0) that never moves
    plt.scatter(0, 0, color='blue', s=100)  # s is the size of the point

    # Calculate arrow direction based on the current angle and make it bigger
    dx = 5 * np.sin(np.radians(arrow_angle))
    dy = 5 * np.cos(np.radians(arrow_angle))
    plt.gca().add_patch(Arrow(0, 0, dx, dy, width=3, color='blue'))  # Increased arrow size

    plt.draw()

'''def shift_points(direction):
    """Shift points based on the arrow key pressed and arrow direction."""
    global points, arrow_angle
    # Calculate shift direction based on arrow orientation
    dx = np.round(6*(np.sin(np.radians(arrow_angle))))
    dy = np.round(6*(np.cos(np.radians(arrow_angle))))

    print(f'dx = {dx}')
    print(f'dy = {dy}')
    print()

    if direction == 'up':
        # Move points in the opposite direction of the arrow
        points = np.roll(points, int(dy), axis=0)
        points = np.roll(points, int(-dx), axis=1)
    elif direction == 'down':
        # Move points in the same direction as the arrow
        points = np.roll(points, int(-dy), axis=0)
        points = np.roll(points, int(dx), axis=1)
    elif direction == 'left':
        arrow_angle -= 10  # Rotate arrow by -10 degrees
    elif direction == 'right':
        arrow_angle += 10  # Rotate arrow by +10 degrees

    plt.clf()
    draw_grid(points, arrow_angle)
'''

def rotate_points(angle, sensor_offset_front=5, wheelbase=7.9):
    """Rotate points around the center of rotation."""
    # Calculate the center of rotation
    global plot_points, center

    center_of_rotation_y = center - (wheelbase / 2.0)

    angle_rad = np.radians(angle)  # positive for counter clockwise rotation
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    # Prepare a new array for rotated points
    new_points = np.zeros_like(plot_points)

    for y in range(grid_size):
        for x in range(grid_size):
            if plot_points[y, x] == 1:
                # Translate point back to the center of rotation
                trans_x = x - center
                trans_y = y - center_of_rotation_y

                # Rotate point
                rotated_x = trans_x * cos_angle - trans_y * sin_angle
                rotated_y = trans_x * sin_angle + trans_y * cos_angle

                # Translate point back to the original place
                new_x = int(rotated_x + center)
                new_y = int(rotated_y + center_of_rotation_y)

                # Check if new position is within bounds
                if 0 <= new_x < grid_size and 0 <= new_y < grid_size:
                    new_points[new_y, new_x] = 1

    plot_points = new_points

def shift_points(direction):
    """Shift points based on the arrow key pressed and arrow direction."""
    global plot_points, arrow_angle
    # Set travel distance based on direction
    rotation = 12.85  # LEFT for 106ms ~=12.85 degrees
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


# Event handler for key presses
def on_key(event):
    if event.key in ['up', 'down', 'left', 'right']:
        shift_points(event.key)

# Set up plot
plt.figure(figsize=(8, 8))
draw_grid(points, arrow_angle)
plt.gcf().canvas.mpl_connect('key_press_event', on_key)

# Display the plot
plt.show()
