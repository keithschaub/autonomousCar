from matplotlib.patches import Rectangle, Polygon, Circle, Arc
import matplotlib.pyplot as plt
import numpy as np
import yaml
import os
import math

# Parameters
grid_size = 1000  # Room size in cm (1000cm x 1000cm)


# Load obstacles and car from YAML
def load_yaml(filename):
    script_dir = os.path.dirname(__file__)
    full_path = os.path.join(script_dir, filename)
    with open(full_path, 'r') as file:
        data = yaml.safe_load(file)
    return data


data = load_yaml('obstacles.yaml')
car_state = data['car']  # Initial car state


# Check if an obstacle is within the radar's field of view
def is_within_radar(car_state, obstacle_center):
    radar_distance = 50  # Radar detection distance in cm
    radar_angle_start = car_state['orientation'] - 60
    radar_angle_end = car_state['orientation'] + 60

    # Calculate angle to obstacle
    dx = obstacle_center[0] - car_state['xcenter']
    dy = obstacle_center[1] - car_state['ycenter']
    angle_to_obstacle = math.degrees(math.atan2(dy, dx)) % 360

    # Check if within radar distance and angle range
    distance_to_obstacle = math.sqrt(dx ** 2 + dy ** 2)
    if distance_to_obstacle <= radar_distance and radar_angle_start <= angle_to_obstacle <= radar_angle_end:
        return True
    return False


# Function to draw the scene (obstacles, car, direction arrow)
# Function to draw the scene (obstacles, car, direction arrow)
def draw_scene(car_state, ax):
    ax.clear()  # Clear previous drawings
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size + 100)  # Extra space for the direction arrow
    ax.set_xticks(np.arange(0, grid_size + 1, 50))
    ax.set_yticks(np.arange(0, grid_size + 1, 50))
    ax.grid(which='both', linestyle='--', linewidth=0.5)

    # Draw obstacles
    for obstacle in data['obstacles']:
        edge_color = 'lightgrey'  # Initial color

        # Calculate the obstacle's center based on its type
        if obstacle['type'] in ['rectangle', 'circle']:
            obstacle_center = (obstacle['xcenter'], obstacle['ycenter'])
        elif obstacle['type'] == 'triangle':
            xs, ys = zip(*obstacle['points'])
            obstacle_center = (sum(xs) / len(xs), sum(ys) / len(ys))

        # Check if the obstacle is within the radar's field of view
        if is_within_radar(car_state, obstacle_center):
            edge_color = 'red'  # Update color if within radar

        # Draw the obstacle with the appropriate color
        if obstacle['type'] == 'rectangle':
            rect = Rectangle(
                (obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2),
                obstacle['width'], obstacle['height'], linewidth=1, edgecolor=edge_color, facecolor='none')
            ax.add_patch(rect)
        elif obstacle['type'] == 'triangle':
            triangle = Polygon(obstacle['points'], linewidth=1, edgecolor=edge_color, facecolor='none')
            ax.add_patch(triangle)
        elif obstacle['type'] == 'circle':
            circle = Circle((obstacle['xcenter'], obstacle['ycenter']), obstacle['radius'], linewidth=1,
                            edgecolor=edge_color,
                            facecolor='none')
            ax.add_patch(circle)

    # Draw the car
    car_rect = Rectangle(
        (car_state['xcenter'] - car_state['width'] / 2, car_state['ycenter'] - car_state['height'] / 2),
        car_state['width'], car_state['height'], linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(car_rect)

    # Draw radar arc
    radar_start_angle = (car_state['orientation'] - 60) % 360  # Adjusting for plot's coordinate system
    radar_extent = 120  # Arc spans from 30 to 150 degrees relative to car's direction
    radar_arc = Arc((car_state['xcenter'], car_state['ycenter']), 100, 100, angle=0, theta1=radar_start_angle,
                    theta2=radar_start_angle + radar_extent, edgecolor='green', linewidth=2)
    ax.add_patch(radar_arc)


# Function to update the car's state based on key presses
def on_key(event):
    global car_state
    if event.key == 'up':
        # Move forward
        orientation_rad = np.radians(car_state['orientation'])
        car_state['xcenter'] += 10 * np.cos(orientation_rad)
        car_state['ycenter'] += 10 * np.sin(orientation_rad)
    elif event.key == 'down':
        # Move backward
        orientation_rad = np.radians(car_state['orientation'])
        car_state['xcenter'] -= 10 * np.cos(orientation_rad)
        car_state['ycenter'] -= 10 * np.sin(orientation_rad)
    elif event.key == 'left':
        # Rotate left
        car_state['orientation'] = (car_state['orientation'] + 90) % 360
    elif event.key == 'right':
        # Rotate right
        car_state['orientation'] = (car_state['orientation'] - 90) % 360

    draw_scene(car_state, ax)  # Redraw the scene with updated car state
    plt.draw()


fig, ax = plt.subplots()
draw_scene(car_state, ax)  # Initial drawing
fig.canvas.mpl_connect('key_press_event', on_key)  # Connect the key press event

plt.show()
