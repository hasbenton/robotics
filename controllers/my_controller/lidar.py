import math
import os
import numpy as np
from controller import Lidar

# ========== Grid Map Configuration ==========
GRID_CELL_SIZE = 0.05    # Each grid cell represents 0.05 meters
MAP_WIDTH = 100          # Grid map width (columns)
MAP_HEIGHT = 100         # Grid map height (rows)
# ==================================

def initialise(init_x, init_y, init_theta, num_particles=20):
    """
    Particle filter initialization function (Resolves 'l.initialise' call error in my_controller)
    :param init_x: Robot initial X coordinate (meters)
    :param init_y: Robot initial Y coordinate (meters)
    :param init_theta: Robot initial orientation (radians)
    :param num_particles: Number of particles (default 20)
    :return: List of initialized particles, each particle is [x, y, theta, weight]
    """
    particles = []
    for _ in range(num_particles):
        x = init_x + np.random.normal(0, 0.05)  # Gaussian noise in X direction
        y = init_y + np.random.normal(0, 0.05)  # Gaussian noise in Y direction
        theta = init_theta + np.random.normal(0, 0.05)  # Gaussian noise in orientation
        weight = 1.0 / num_particles  # Initial weight is equally distributed
        particles.append([x, y, theta, weight])
    return particles

def meters_to_grid(meters):
    """Meters to grid coordinates (Core conversion function)"""
    # Assuming (MAP_WIDTH/2, MAP_HEIGHT/2) is the origin (0,0) in meters
    return int(meters / GRID_CELL_SIZE + MAP_WIDTH/2)

def run_lidar(motion, robot_x, robot_y, robot_theta, lidar, particles):
    """
    LiDAR data processing: Generates a grid map (marking obstacles)
    :param motion: [Distance moved, Angle change]
    :param robot_x/robot_y/robot_theta: Robot's current pose
    :param lidar: LiDAR device instance
    :param particles: Particle filter (used for localization correction, though not fully implemented in this function's current use)
    :return: The updated grid map
    """
    # Initialize an empty grid map (0=Free, 1=Obstacle)
    grid = [[0 for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]
    
    # Get LiDAR range data
    ranges = lidar.getRangeImage()
    num_points = len(ranges)
    min_range = lidar.getMinRange()
    max_range = lidar.getMaxRange()
    
    # LiDAR angular range: -π ~ π (360° scan)
    total_angle = 2 * math.pi
    angle_step = total_angle / num_points
    
    # Iterate through all laser points and mark obstacles
    for i in range(num_points):
        distance = ranges[i]
        # Filter invalid data (outside LiDAR detection range)
        if not (math.isfinite(distance) and min_range < distance < max_range):
            continue
        
        # Calculate the laser point's global angle
        laser_angle = -math.pi + i * angle_step + robot_theta
        # Calculate the obstacle's global coordinates
        obs_x = robot_x + distance * math.cos(laser_angle)
        obs_y = robot_y + distance * math.sin(laser_angle)
        
        # Convert to grid coordinates and mark
        grid_x = meters_to_grid(obs_x)
        grid_y = meters_to_grid(obs_y)
        if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
            grid[grid_y][grid_x] = 1
    
    return grid

def save_grid_map(grid):
    """Save the grid map to the controller directory (Generates grid_map.txt)"""
    file_path = os.path.join(os.path.dirname(__file__), "grid_map.txt")
    with open(file_path, "w", encoding="utf-8") as f:
        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")
    print(f"✅ Grid map saved: {os.path.abspath(file_path)}")

def visualize_grid(grid):
    """Console visualization of the center 30x30 grid (To view obstacles)"""
    print("\n📊 Grid Map Center 30x30 Area (1=Obstacle):")
    start = 35
    end = 65
    for y in range(start, end):
        print(" ".join(map(str, grid[y][start:end])))
