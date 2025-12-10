import numpy as np
import math
import random as r
from copy import deepcopy

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
    grid_x = int(round(abs_x / 0.05))
    grid_y = int(round(abs_y / 0.05))
    
    grid_x += 50
    grid_y += 50

    grid_x = max(grid_x,0)
    grid_x = min(grid_x, 99)

    grid_y = max(grid_y,0)
    grid_y = min(grid_y,99)

    return grid_x, grid_y

def Particle_filter(particles, movement, sensor_read, map) :
    '''Using Monte Carlo Localisation.
    
    Assume particles is a list of (X, Y , Facing, probability of correct position)

    movement is  (distance, rotation) - Robot rotated then moved

    sensor_read is an array of points

    map is a 2d array'''
    #                                 Mean, Standard Distribution, Size of array
    move_random = movement[0] + (np.random.normal(0.0, 1.0, len(particles))  * movement[0] / 20) #20 is how much randomness affects the movements 
    rotate_random = movement[1] + (np.random.normal(0.0, 1.0, len(particles)) * movement[1] / 20)#20 Can be changed
    
    #Update positions with moving
    for i in range(len(particles)) :

        #Rotation
        particles[i][2] += rotate_random[i]

        #x-coord
        particles[i][0] += move_random[i] * math.cos(particles[i][2])
        #y-coord
        particles[i][1] += move_random[i] * math.sin(particles[i][2])

        
    
    #Weight each point based off of sensor data
    weighted_particles = []
    for point in particles :
        weight = 1
        #Check each direction compared to the map
        for i in range(len(sensor_read)) :
            
            if math.isinf(sensor_read[i].x) or math.isinf(sensor_read[i].y) :
                 continue
            if math.isnan(sensor_read[i].y) or math.isnan(sensor_read[i].y) :
                 continue
            
            position = [point[0],point[1]] #[x,y]
            
            #Converts sensor point to a point on the grid based off of the particle's position and facing
            sensor = lidar_point_to_grid(sensor_read[i].x, sensor_read[i].y, point[2], position[0], position[1])
            
            if map[-sensor[1]][sensor[0]] == 1 :
                 weight *= 1.2
            else :
                weight *= 0.5

        #new weight
        point[3] = weight
        weighted_particles.append(weight)

    
    #Now sample new points based off of these weighted points
    weight_sum = sum(weighted_particles)

    #Turn all weights into percentages
    for i in range(len(particles)) :
         particles[i][3] /= weight_sum
         weighted_particles[i] /= weight_sum


    newParticles = []



    for i in range(len(particles)) :
         pick = r.random()
         for j in range(len(weighted_particles)) :
              pick -= weighted_particles[j] 
              if pick <= 0 :
                   
                   newParticles.append(deepcopy(particles[j]))
                   break
              


    particles = newParticles

    return (particles)
