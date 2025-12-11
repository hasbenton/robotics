import math
import os
import numpy as np
from controller import Lidar
from particle_filter import Particle_filter
import sys

# ========== Grid Map Configuration ==========
GRID_CELL_SIZE = 0.05    # Each grid cell represents 0.25 meters
MAP_WIDTH = 100    # Grid map width (columns)
MAP_HEIGHT = 100    # Grid map height (rows)

global_map = None    # Global accumulated map (empty initially)
# ==================================

def initialise(init_x, init_y, init_theta, num_particles=50):
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
        x = init_x + np.random.normal(0, 0.02)  # Gaussian noise in X direction
        y = init_y + np.random.normal(0, 0.02)  # Gaussian noise in Y direction
        theta = init_theta + np.random.normal(0, 0.05)  # Gaussian noise in orientation
        weight = 1.0 / num_particles  # Initial weight is equally distributed
        particles.append([x, y, theta, weight])
    return particles

def meters_to_grid(meters):
    """Meters to grid coordinates (Core conversion function)"""
    # Assuming (MAP_WIDTH/2, MAP_HEIGHT/2) is the origin (0,0) in meters
    return int(meters / GRID_CELL_SIZE + MAP_WIDTH/2)

def grid_to_meters(grid_index):
    """[New] Grid index -> Meters (Used to convert A* path points back to world coordinates)"""
    return (grid_index - MAP_WIDTH/2) * GRID_CELL_SIZE

def run_lidar(motion, robot_x, robot_y, robot_theta, lidar, particles, grid):
    """
    LiDAR data processing: Generates a grid map (marking obstacles)
    :param motion: [Distance moved, Angle change]
    :param robot_x/robot_y/robot_theta: Robot's current pose
    :param lidar: LiDAR device instance
    :param particles: Particle filter
    :param grid: Current grid map
    :return: The updated grid map, updated particles, and estimated pose
    """
    
    # [New] 1. Must first get LiDAR point cloud data, define point_cloud variable
    point_cloud = lidar.getPointCloud()
    grid_x = 0
    grid_y = 0
    # 3. [Core] Run Particle Filter (SLAM Localization Correction)
    # This step corrects particle positions based on LiDAR data
    # Pass in the grid generated in the previous frame (as map reference); can be empty for the first frame
    # Note: This is a simplification; actual SLAM requires a persistent global map
    # Here, we primarily use it for Localization
    if len(point_cloud) > 0:
        particles = Particle_filter(particles, motion, point_cloud, grid)
    
    # 4. [Core] Calculate Best Estimated Position (Weighted average of all particles)
    # This is the "true" position after error elimination
    est_x = sum(point[0] * point[3] for point in particles) / sum(point[3] for point in particles)
    est_y = sum(point[1] * point[3] for point in particles) / sum(point[3] for point in particles)
    est_theta = - sum(point[2] * point[3] for point in particles) / sum(point[3] for point in particles)
    est_pos = [est_x, est_y, est_theta]


    # Get LiDAR range data
    ranges = lidar.getRangeImage()
    num_points = len(ranges)
    min_range = lidar.getMinRange()
    max_range = lidar.getMaxRange()
    
    # LiDAR angular range: -π ~ π (360° scan)
    total_angle = 2 * math.pi
    angle_step = total_angle / num_points

    for point in point_cloud:
        x,y = [point.x, point.y]

        if (math.isfinite(x) or math.isfinite(y)):
            distance = math.sqrt((x*x) + (y*y))
            if min_range < distance < max_range:
                x = x * math.cos(robot_theta) - y * math.sin(robot_theta)
                y = x * math.sin(robot_theta) + y * math.cos(robot_theta)

                x += robot_x
                y -= robot_y

                grid_x = meters_to_grid(x)
                grid_y = meters_to_grid(y)
                if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
                     grid[-grid_y][grid_x] = 1

    
    # # Iterate through all laser points and mark obstacles
    # for i in range(num_points):
    #     distance = ranges[i]
    #     # Filter invalid data (outside LiDAR detection range)
    #     if not (math.isfinite(distance) and min_range < distance < max_range):
    #         continue
        
    #     # Calculate the laser point's global angle
    #     laser_angle = -math.pi + i * angle_step + est_theta
    #     # Calculate the obstacle's global coordinates
    #     obs_x = est_x + distance * math.cos(laser_angle)
    #     obs_y = est_y + distance * math.sin(laser_angle)
        
    #     # Convert to grid coordinates and mark
    #     grid_x = meters_to_grid(obs_x)
    #     grid_y = meters_to_grid(obs_y)
    #     if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
    #         grid[grid_y][grid_x] = 1
    
    grid_x = meters_to_grid(est_x)
    grid_y = meters_to_grid(est_y)

    # if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
    #     grid[grid_y][grid_x] = 2

    return grid, particles, est_pos

def save_grid_map(grid):
    return
    """Save the grid map to the controller directory (Generates grid_map.txt)"""
    file_path = os.path.join(os.path.dirname(__file__), f"grid_map{sys.argv[1]}.txt")
    with open(file_path, "w", encoding="utf-8") as f:
        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")
    print(f"✅ Grid map saved: {os.path.abspath(file_path)}")

def visualize_grid(grid):
    return
    """Console visualization of the center 30x30 grid (To view obstacles)"""
    print("\n📊 Grid Map Center 30x30 Area (1=Obstacle):")
    start = 35
    end = 65
    for y in range(start, end):
        print(" ".join(map(str, grid[y][start:end])))