# All my imports ( the legal ones anyway :) )
import sys
import matplotlib.pyplot as plt
import RRT
import obstaclefinder
import math
import numpy as np

# This function calculates the configurations space and runs the RRT
def runRRT(dynamics, robotSize, data ,start = [0,50],end = [1950,1650]):
    data = data[::-1, :]
    plt.imshow(data.T, interpolation='nearest')  # show 2D representation of map
    img = obstaclefinder.imgToObs()  # create img from imported picture
    if dynamics == 'HOUND':
        scaleFactor = 3
    elif dynamics == 'HIPPO':
        scaleFactor = 6
    obs = img.obsSpaceGen(robotSize, data, scaleFactor, debug=False)
    plt.imshow(data,interpolation='nearest') #show 2D representation of map
    
    #############################################################################################
    """
        #SET THE GOAL AND CAR THAT IS BEING TESTED
        goal = Goal, may alter first three values for x,y,theta (DO NOT ALTER REMAINING VALUES)
            x and y are bound from 0 to 1199 and theta is bound to -math.pi and math.pi
        HD = True if Hound running the Maze, eitherwise false
        HP = True if Hippo running the Maze, eitherwise false
        NOTE: Both should not be true, can be done but with no collision avoidance, will fail
    """
    goal = [800,900, 0]
    HD = False
    HP = True
    
    #############################################################################################
    
    if dynamics == 'HOUND' and not HD:
        return [[0,0,0,[[0,0,100]]]]
    if dynamics == 'HIPPO' and not HP:
        return [[0,0,0,[[0,0,0,0,100]]]]
    
    #initialize RRT
    #print(obs[0].shape)
    r = RRT.rrt(N = 5000,obstacles = obs.T, obstacletype = 'array', maxcoords = obs[0].shape,
            origin = start+[0,'',0],goal = goal, live = False, divis = 10)
            
    #Perform RRT
    trajectory = r.rrt(dynamics,plotting=True)

    return trajectory

#(verbose = True, plotting=True, dynamics)