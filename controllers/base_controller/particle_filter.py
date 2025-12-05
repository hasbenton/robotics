import numpy as np
import math
import random as r
from lidar import lidar_point_to_grid


def Particle_filter(particles, movement, sensor_read, map) :
    '''Using Monte Carlo Localisation.
    
    Assume particles is a list of (Co-ords, Facing, probability of correct position)

    movement is  (distance, rotation) - Robot rotated then moved

    sensor_read is an array of points

    map is a 2d array'''
    #                                 Mean, Standard Distribution, Size of array
    move_random = movement[0] + (np.random.normal(0.0, 1.0, len(particles))  * movement[0] / 20) #20 is how much randomness affects the movements 
    rotate_random = movement[1] + (np.random.normal(0.0, 1.0, len(particles)) * movement[1] / 20)#20 Can be changed

    #Update positions with moving
    for i in range(len(particles)) :
        #x-coord
        particles[i][0][0] += move_random[i] * math.cos(rotate_random[i])
        #y-coord
        particles[i][0][1] += move_random[i] * math.sin(rotate_random[i])
        #Rotation
        particles[i][1] += rotate_random[i]
    
    #Weight each point based off of sensor data
    weighted_particles = []
    for point in particles :
        sum_of_squares = 0
        #Check each direction compared to the map
        for i in range(len(sensor_read)) :
            
            hit1 = False
            position = point[0] #[x,y]
            
            #Converts sensor point to a point on the grid based off of the particle's position and facing
            sensor = lidar_point_to_grid(sensor_read[i].x, sensor_read[i].y, point[1], position[0], position[1])
            

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
                 distance = math.sqrt(sum(x*x for x in position))

                 sum_of_squares += math.sqrt(
                           (sensor_read[i] * sensor_read[i]) + (distance * distance)
                      )

        point[2] = 1 - sum_of_squares
        #1 / sum_of_squares is to give smaller differences more weight
        weighted_particles.append(1 / sum_of_squares)

    #Now sample new points based off of these weighted points
    weight_sum = sum(weighted_particles)
    newParticles = []
    for i in range(particles) :
         pick = r.random() * weight_sum

         for j in range(len(weighted_particles)) :
              pick -= weighted_particles[j]

              if pick < 0 :
                   newParticles.append(particles[j])

    particles = newParticles

    return (particles)
