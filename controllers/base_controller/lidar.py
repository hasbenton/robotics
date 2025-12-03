from controller import Robot
import math

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Configure LiDAR (matches device name in rosbot.c)
lidar = robot.getDevice('laser')
lidar.enable(timestep)
lidar.enablePointCloud()  # Get 3D points (x, y, z)

# Known robot position in meters (e.g., (1.0, 1.0) meters)
ROBOT_X = 1.0  # x-coordinate in meters
ROBOT_Y = 1.0  # y-coordinate in meters
ROBOT_THETA = 0.0  # Orientation (radians, 0 = facing positive x)

# Grid parameters (matches test_maps.py: 22x22 grid, 0.25m per cell)
GRID_CELL_SIZE = 0.25  # Meters per grid cell
GRID_WIDTH = 22        # Number of columns
GRID_HEIGHT = 22       # Number of rows

# Initialize grid with map1 (from test_maps.py) as a base
from test_maps import map1
grid = [row.copy() for row in map1]

def meters_to_grid(meters):
    """Convert meters to grid cell indices (integer)."""
    return int(round(meters / GRID_CELL_SIZE))

def lidar_point_to_grid(px, py):
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

# Main loop to update grid with LiDAR data
while robot.step(timestep) != -1:
    # Get LiDAR point cloud (list of (x, y, z) relative to robot)
    point_cloud = lidar.getPointCloud()
    
    for point in point_cloud:
        px, py, pz = [point.x,point.y,point.z]  # pz is height (ignore for 2D mapping)
        py = - py #As the map goes down for y-coordinates
        distance = math.hypot(px, py)  # Distance from robot to point
        
        # Filter valid wall points (0.1m to 5m to avoid noise/errors)
        if 0.1 < distance < 5.0:
            grid_x, grid_y = lidar_point_to_grid(px, py)
            
            # Update grid if within bounds
            if 0 <= grid_x < GRID_WIDTH and 0 <= grid_y < GRID_HEIGHT:
                grid[grid_y][grid_x] = 1  # Mark as wall (1)
               