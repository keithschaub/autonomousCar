import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arc, PathPatch
from matplotlib.path import Path
import os
import yaml
import math

# Parameters
grid_size = 1000  # Room size in cm (1000cm x 1000cm)

# Global dictionary to store discovered hit points for each edge
discovered_edges = {}  # Initialize as empty dictionary

# Load obstacles and car from YAML
def load_yaml(filename):
    script_dir = os.path.dirname(__file__)  # Adjust if necessary
    full_path = os.path.join(script_dir, filename)
    with open(full_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

data = load_yaml('obstacles.yaml')  # Ensure 'obstacles.yaml' is in the correct directory
car_state = data['car']  # Initial car state

def edge_identifier(obstacle_index, edge_index):
    """Generate a unique identifier for each edge."""
    return f"{obstacle_index}-{edge_index}"

def line_intersection(line1, line2):
    """Check if line1 intersects with line2 and return the intersection point if it does."""
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None  # Lines are parallel

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    # Check if the intersection is within the bounds of the radar vector
    if min(line1[0][0], line1[1][0]) <= x <= max(line1[0][0], line1[1][0]) and min(line1[0][1], line1[1][1]) <= y <= max(line1[0][1], line1[1][1]):
        # Check if the intersection is within the bounds of the obstacle edge
        if min(line2[0][0], line2[1][0]) <= x <= max(line2[0][0], line2[1][0]) and min(line2[0][1], line2[1][1]) <= y <= max(line2[0][1], line2[1][1]):
            return (x, y)
    return None


def update_discovered_edges(edge_id, hit_point):
    """Update the list of hit points for a given edge."""
    if edge_id not in discovered_edges:
        discovered_edges[edge_id] = []
    discovered_edges[edge_id].append(hit_point)
    print(f"Updated discovered_edges: {discovered_edges}")  # Debugging

def draw_hit_points(ax):
    """Draw red dots for all discovered hit points."""
    for edge_id, points in discovered_edges.items():
        for point in points:
            ax.plot(point[0], point[1], 'ro', markersize=1)

def radar_sweep(car_state, ax, data):
    radar_distance = 50  # Radar detection distance in cm
    orientation_deg = car_state['orientation']
    start_angle_deg = orientation_deg - 60  # Start 30 degrees to the left of car's orientation
    end_angle_deg = orientation_deg + 60  # End 150 degrees to the right of car's orientation
    step_size_deg = 5

    for angle_deg in range(start_angle_deg, end_angle_deg + 1, step_size_deg):
        angle_rad = np.radians(angle_deg)
        radar_vector_end = (car_state['xcenter'] + radar_distance * np.cos(angle_rad),
                            car_state['ycenter'] + radar_distance * np.sin(angle_rad))
        radar_vector = [(car_state['xcenter'], car_state['ycenter']), radar_vector_end]

        for i, obstacle in enumerate(data['obstacles']):
            if obstacle['type'] == 'rectangle':
                # Define rectangle edges based on the obstacle properties
                rectangle_edges = [
                    [(obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2),
                     (obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2)],  # Left edge
                    [(obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2),
                     (obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2)],  # Top edge
                    [(obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] + obstacle['height'] / 2),
                     (obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2)],  # Right edge
                    [(obstacle['xcenter'] + obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2),
                     (obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2)]   # Bottom edge
                ]

                for edge_index, edge in enumerate(rectangle_edges):
                    intersection = line_intersection(radar_vector, edge)
                    if intersection:
                        # Calculate distance from car to intersection point
                        dist_to_hit = np.sqrt((intersection[0] - car_state['xcenter'])**2 + (intersection[1] - car_state['ycenter'])**2)
                        if dist_to_hit <= radar_distance:
                            print(dist_to_hit, radar_distance)
                            edge_id = edge_identifier(i, edge_index)
                            update_discovered_edges(edge_id, intersection)
                            # Draw hit point
                            ax.plot(intersection[0], intersection[1], 'ro', markersize=2)
                            # Adjust the radar vector to end at the hit point
                            ax.plot([car_state['xcenter'], intersection[0]], [car_state['ycenter'], intersection[1]], 'g-', linewidth=1)
                            break  # Found an intersection, no need to check other edges of this obstacle

        # Draw the radar vector if no intersection was found
        ax.plot([car_state['xcenter'], radar_vector_end[0]], [car_state['ycenter'], radar_vector_end[1]], 'g-', linewidth=1)


def draw_scene(car_state, data, fig, ax):
    ax.clear()  # Clear the axes to redraw the scene
    ax.set_aspect('equal', 'box')
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_xticks(np.arange(0, grid_size + 1, 50))
    ax.set_yticks(np.arange(0, grid_size + 1, 50))
    ax.grid(which='both', linestyle='--', linewidth=0.5)

    # Draw obstacles as rectangles
    for obstacle in data['obstacles']:
        if obstacle['type'] == 'rectangle':
            rect = Rectangle((obstacle['xcenter'] - obstacle['width'] / 2, obstacle['ycenter'] - obstacle['height'] / 2),
                             obstacle['width'], obstacle['height'], linewidth=1, edgecolor='black', facecolor='none')
            ax.add_patch(rect)

    # Draw the car
    car_rect = Rectangle((car_state['xcenter'] - car_state['width'] / 2, car_state['ycenter'] - car_state['height'] / 2),
                         car_state['width'], car_state['height'], linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(car_rect)

    # Perform and visualize the radar sweep
    radar_sweep(car_state, ax, data)

    # Draw hit points for radar detection
    draw_hit_points(ax)

    plt.draw()

def on_key(event, fig, ax, car_state, data):
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
    draw_scene(car_state, data, fig, ax)


def main():
    global data, car_state
    fig, ax = plt.subplots()
    draw_scene(car_state, data, fig, ax)
    callback = lambda event: on_key(event, fig, ax, car_state, data)
    fig.canvas.mpl_connect('key_press_event', callback)
    plt.show()

if __name__ == "__main__":
    main()