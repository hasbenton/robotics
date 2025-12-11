import math
import os
import numpy as np
from controller import Lidar
from particle_filter import Particle_filter
import sys

# Grid Map Configuration
GRID_CELL_SIZE = 0.05    # Each grid cell in meters
MAP_WIDTH = 100    # Grid map width (columns)
MAP_HEIGHT = 100    # Grid map height (rows)

global_map = None    # Global accumulated map (initially empty)

def initialise(init_x, init_y, init_theta, num_particles=50):
    """Initialize particle filter with Gaussian noise; returns list of particles [x, y, theta, weight]"""
    particles = []
    for _ in range(num_particles):
        x = init_x + np.random.normal(0, 0.02)
        y = init_y + np.random.normal(0, 0.02)
        theta = init_theta + np.random.normal(0, 0.05)
        weight = 1.0 / num_particles
        particles.append([x, y, theta, weight])
    return particles

def meters_to_grid(meters):
    """Convert meters to grid coordinates (origin at map center)"""
    return int(meters / GRID_CELL_SIZE + MAP_WIDTH/2)

def grid_to_meters(grid_index):
    """Convert grid index to meters"""
    return (grid_index - MAP_WIDTH/2) * GRID_CELL_SIZE

def run_lidar(motion, robot_x, robot_y, robot_theta, lidar, particles, grid):
    """Process LiDAR data to update grid map and particle filter; returns updated grid, particles, and estimated pose"""
    point_cloud = lidar.getPointCloud()
    grid_x, grid_y = 0, 0

    # Update particles using particle filter if point cloud is valid
    if len(point_cloud) > 0:
        particles = Particle_filter(particles, motion, point_cloud, grid)
    
    # Calculate estimated position from particle weights
    sum_weights = sum(point[3] for point in particles)
    est_x = sum(point[0] * point[3] for point in particles) / sum_weights
    est_y = sum(point[1] * point[3] for point in particles) / sum_weights
    est_theta = -sum(point[2] * point[3] for point in particles) / sum_weights
    est_pos = [est_x, est_y, est_theta]

    # Process LiDAR point cloud to mark obstacles
    ranges = lidar.getRangeImage()
    num_points = len(ranges)
    min_range = lidar.getMinRange()
    max_range = lidar.getMaxRange()
    
    total_angle = 2 * math.pi
    angle_step = total_angle / num_points

    for point in point_cloud:
        x, y = point.x, point.y
        if math.isfinite(x) or math.isfinite(y):
            distance = math.sqrt(x**2 + y**2)
            if min_range < distance < max_range:
                # Rotate coordinates based on robot orientation
                x_rot = x * math.cos(robot_theta) - y * math.sin(robot_theta)
                y_rot = x * math.sin(robot_theta) + y * math.cos(robot_theta)
                # Transform to world coordinates
                x_rot += robot_x
                y_rot += robot_y

                grid_x = meters_to_grid(x_rot)
                grid_y = meters_to_grid(y_rot)
                if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
                    grid[-grid_y][grid_x] = 1  # Mark obstacle

    # Mark robot's estimated position
    grid_x = meters_to_grid(est_x)
    grid_y = meters_to_grid(est_y)
    if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
        grid[grid_y][grid_x] = 2  # Mark robot position

    return grid, particles, est_pos

def save_grid_map(grid):
    """Save grid map to file"""
    file_path = os.path.join(os.path.dirname(__file__), f"grid_map{sys.argv[1]}.txt")
    with open(file_path, "w", encoding="utf-8") as f:
        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")
    print(f"✅ Grid map saved: {os.path.abspath(file_path)}")

def visualize_grid(grid):
    """Visualize center 30x30 grid area"""
    print("\n📊 Grid Map Center 30x30 Area (1=Obstacle):")
    start, end = 35, 65
    for y in range(start, end):
        print(" ".join(map(str, grid[y][start:end])))
