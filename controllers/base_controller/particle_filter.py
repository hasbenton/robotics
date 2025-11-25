import numpy as np
import math
import random as r

#Using Monte Carlo Localisation
#Assume particles is a list of (Co-ords, Facing, probability of correct position)
#       movement is  (distance, rotation) - Robot rotated then moved
#       sensor_read is an array of distances
#       map is a 2d array
def Particle_filter(particles, movement, sensor_read, map) :
    #                                   Mean, Standard Distribution, Size of array
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
            #Rotate sensor data based off of orientation
            angle = (i * point[1] * 2 * math.pi) / len(sensor_read)

            #Go in the direction of the sensor until a 1 is found on the map
            hit1 = False
            position = point[0] #[x,y]

            #If not hit and position inside the map 
            while (not(hit1) and 0 <= position[0] <= len(map[0]) and 0 <= position[1] <= len(map)) :
                   
                   if (map[round(position[0])][round(position[1])] == 1) :
                        hit1 = True
                        break
                   
                   else :
                        #Move to the next position
                        #Shortest distance to make it to the next coord in x or y
                        #By reverse polar positions - doing Update Positions thing backwards

                        #Works out if angle is going positive or negative for x
                        x = 0.5 if (- math.pi / 2) < angle < (math.pi / 2) else -0.49
                        x += round(position[0])

                        x = (x - position[0]) / math.cos(angle)
                        #x is now a scaling factor

                        #Works out if angle is going positive or negative for y
                        y = 0.5 if 0 < angle < math.pi else -0.49
                        y += round(position[1])

                        y = (y - position[1]) / math.sin(angle)
                        #y is now a scaling factor

                        if x > y :
                             position[0] += y * math.cos(angle)
                             position[1] += y * math.sin(angle)
                        else :
                             position[0] += x * math.cos(angle)
                             position[1] += x * math.sin(angle)
            
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
