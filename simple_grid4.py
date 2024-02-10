from matplotlib.patches import Rectangle, Polygon, Circle, Arc, PathPatch
from matplotlib.path import Path
import matplotlib.pyplot as plt
import numpy as np
import yaml
import os
import math

# Parameters
grid_size = 1000  # Room size in cm (1000cm x 1000cm)

# Load obstacles and car from YAML
def load_yaml(filename):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(script_dir, filename)
    with open(full_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

data = load_yaml('obstacles.yaml')
car_state = data['car']  # Initial car state
discovered_edges = {}  # Track discovered obstacle edges by their unique identifiers

def edge_identifier(obstacle_index, edge_index):
    """Generate a unique identifier for each edge."""
    return f"{obstacle_index}-{edge_index}"

def point_within_radar(car_state, point):
    """Check if a point is within the radar's field of view."""
    radar_distance = 50  # Radar detection distance in cm
    orientation_rad = np.radians(car_state['orientation'])
    radar_start_angle = orientation_rad - np.radians(60)
    radar_end_angle = orientation_rad + np.radians(60)

    dx = point[0] - car_state['xcenter']
    dy = point[1] - car_state['ycenter']
    distance = math.sqrt(dx**2 + dy**2)
    angle_to_point = math.atan2(dy, dx)

    # Normalize angles
    if angle_to_point < 0:
        angle_to_point += 2 * np.pi

    return (distance <= radar_distance and
            radar_start_angle <= angle_to_point <= radar_end_angle)

def draw_scene(car_state, ax):
    ax.clear()
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_xticks(np.arange(0, grid_size + 1, 50))
    ax.set_yticks(np.arange(0, grid_size + 1, 50))
    ax.grid(which='both', linestyle='--', linewidth=0.5)

    for i, obstacle in enumerate(data['obstacles']):
        if obstacle['type'] == 'rectangle':
            # Define rectangle edges
            edges = [
                [(obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2),
                 (obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2)],  # Top edge
                [(obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2),
                 (obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2)],  # Bottom edge
                [(obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2),
                 (obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2)],  # Left edge
                [(obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2),
                 (obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2)]   # Right edge
            ]

            for edge_index, edge in enumerate(edges):
                edge_id = edge_identifier(i, edge_index)
                if edge_id in discovered_edges:
                    edge_color = 'red'
                else:
                    edge_color = 'lightgrey'
                    if point_within_radar(car_state, edge[0]) or point_within_radar(car_state, edge[1]):
                        discovered_edges[edge_id] = True
                        edge_color = 'red'

                # Draw the edge
                line = PathPatch(Path(edge), edgecolor=edge_color, lw=2, facecolor='none')
                ax.add_patch(line)

    # Draw the car
    car_rect = Rectangle((car_state['xcenter'] - car_state['width'] / 2, car_state['ycenter'] - car_state['height'] / 2),
                         car_state['width'], car_state['height'], linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(car_rect)

    # Draw radar arc
    radar_arc = Arc((car_state['xcenter'], car_state['ycenter']), 100, 100, angle=0, theta1=car_state['orientation'] - 60,
                    theta2=car_state['orientation'] + 60, edgecolor='green', linewidth=2)
    ax.add_patch(radar_arc)

def on_key(event):
    global car_state
    if event.key == 'up':
        car_state['xcenter'] += 10 * np.cos(np.radians(car_state['orientation']))
        car_state['ycenter'] += 10 * np.sin(np.radians(car_state['orientation']))
    elif event.key == 'down':
        car_state['xcenter'] -= 10 * np.cos(np.radians(car_state['orientation']))
        car_state['ycenter'] -= 10 * np.sin(np.radians(car_state['orientation']))
    elif event.key == 'left':
        car_state['orientation'] = (car_state['orientation'] + 90) % 360
    elif event.key == 'right':
        car_state['orientation'] = (car_state['orientation'] - 90) % 360
    draw_scene(car_state, ax)
    plt.draw()

fig, ax = plt.subplots()
draw_scene(car_state, ax)
fig.canvas.mpl_connect('key_press_event', on_key)
plt.show()
