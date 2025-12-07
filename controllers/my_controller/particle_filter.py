import numpy as np
import math
import random as r

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
    grid_x = int(round(abs_x / 0.25))
    grid_y = int(round(abs_y / 0.25))
    
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
        #x-coord
        particles[i][0] += move_random[i] * math.cos(rotate_random[i])
        #y-coord
        particles[i][1] += move_random[i] * math.sin(rotate_random[i])
        #Rotation
        particles[i][2] += rotate_random[i]
    
    #Weight each point based off of sensor data
    weighted_particles = []
    for point in particles :
        sum_of_squares = 0
        #Check each direction compared to the map
        for i in range(len(sensor_read)) :
            
            if math.isinf(sensor_read[i].x) or math.isinf(sensor_read[i].y) :
                 continue
            if math.isnan(sensor_read[i].y) or math.isnan(sensor_read[i].y) :
                 continue
            
            hit1 = False
            position = [point[0],point[1]] #[x,y]
            
            #Converts sensor point to a point on the grid based off of the particle's position and facing
            sensor = lidar_point_to_grid(sensor_read[i].x, sensor_read[i].y, point[2], position[0], position[1])
            

            #If not hit and position inside the map 
            while (not(hit1) and 0 <= position[0] <= len(map[0]) and 0 <= position[1] <= len(map)) :
                   
                   if (map[round(position[0])][round(position[1])] == 1) :
                        hit1 = True
                        break
                   
                   else :
                        #Move to the next position
                        #Shortest distance to make it to the next coord in x or y
                        
                         if (sensor[0] > 0) :
                              x = ((round(position[0]) - 0.5) - position[0]) / sensor[0]
                         else :
                              x = ((round(position[0]) + 0.5) - position[0]) / sensor[0]

                         if (sensor[1] > 0) :
                              y = ((round(position[1]) - 0.5) - position[1]) / sensor[1]
                         else :
                              y = ((round(position[1]) + 0.5) - position[1]) / sensor[1]

                         if x < y :
                              position[0] += x * sensor[0]
                              position[1] += x * sensor[1]
                         else :
                              position[0] += y * sensor[0]
                              position[1] += y * sensor[1]

                                  
            
            #If hit wall, find distance and find difference to sensor read
            if hit1 :
                 distance = [abs(sensor_read[i].x - position[0]), abs(sensor_read[i].y - position[1])] 

                 sum_of_squares +=  math.sqrt(sum(x*x for x in distance))

        point[3] = 1 - sum_of_squares
        #1 / sum_of_squares is to give smaller differences more weight
        if (sum_of_squares != 0) :
              weighted_particles.append(1 / sum_of_squares)
        else :
              weighted_particles.append(0)


    #Now sample new points based off of these weighted points
    weight_sum = sum(weighted_particles)
    newParticles = []
    for i in range(len(particles)) :
         pick = r.random() * weight_sum

         for j in range(len(weighted_particles)) :
              pick -= weighted_particles[j] 
              if pick <= 0 :
                   newParticles.append(particles[j])


    particles = newParticles

    return (particles)
