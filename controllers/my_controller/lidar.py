from controller import Robot
from particle_filter import Particle_filter
import math

# Grid parameters (matches test_maps.py: 22x22 grid, 0.25m per cell)
GRID_CELL_SIZE = 0.25  # Meters per grid cell
GRID_WIDTH = 22        # Number of columns
GRID_HEIGHT = 22       # Number of rows

# Initialize grid with map1 (from test_maps.py) as a base
from test_maps import map1
grid = [row.copy() for row in map1]


def initialise(ROBOT_X, ROBOT_Y, ROBOT_THETA) :
    """Initialise Particle filter map"""

    PARTICLES = [[ROBOT_X/GRID_CELL_SIZE, ROBOT_Y/GRID_CELL_SIZE, ROBOT_THETA, 1] for i in range(20)]

    return PARTICLES

def meters_to_grid(meters):
    """Convert meters to grid cell indices (integer)."""
    return int(round(meters / GRID_CELL_SIZE))

def lidar_point_to_grid(px, py, ROBOT_THETA, ROBOT_X, ROBOT_Y):
    """
    Convert LiDAR point (relative to robot) to grid coordinates.
    Accounts for robot's position and orientation.
    """
    # Rotate point by robot's orientation (theta)
    rotated_x = px * math.cos(ROBOT_THETA) - py * math.sin(ROBOT_THETA)
    rotated_y = px * math.sin(ROBOT_THETA) + py * math.cos(ROBOT_THETA)
    
    # Convert to absolute position (world coordinates)
    abs_x = ROBOT_X + rotated_x
    abs_y = ROBOT_Y + rotated_y
    
    # Convert to grid coordinates
    grid_x = meters_to_grid(abs_x)
    grid_y = meters_to_grid(abs_y)
    
    return grid_x, grid_y

# 
def run_lidar(movement, ROBOT_X, ROBOT_Y, ROBOT_THETA, lidar, PARTICLES):
    """Main loop to update grid with LiDAR data"""
    
    # Get LiDAR point cloud (list of (x, y, z) relative to robot)
    point_cloud = lidar.getPointCloud()

    PARTICLES = Particle_filter(PARTICLES, movement, point_cloud, grid)

    #Uses highest probability to update position
    temp = [x[3] for x in PARTICLES]
    best = temp.index(max(temp))

    ROBOT_X = PARTICLES[best][0]
    ROBOT_Y = PARTICLES[best][1]
    ROBOT_THETA = PARTICLES[best][2]
    
    for point in point_cloud:
        px, py, pz = [point.x,point.y,point.z]  # pz is height (ignore for 2D mapping)
        py = - py #As the map goes down for y-coordinates
        distance = math.hypot(px, py)  # Distance from robot to point


        
        # Filter valid wall points (0.1m to 5m to avoid noise/errors)
        if 0.1 < distance < 5.0:
            grid_x, grid_y = lidar_point_to_grid(px, py, ROBOT_THETA, ROBOT_X, ROBOT_Y)
            
            # Update grid if within bounds
            if 0 <= grid_x < GRID_WIDTH and 0 <= grid_y < GRID_HEIGHT:
                grid[grid_y][grid_x] = 1  # Mark as wall (1)

    for c in grid :
        print(c)

    return grid
               