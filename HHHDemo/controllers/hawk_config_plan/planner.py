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
        
        
    ##########################################################################################
    """
        THIS SECTION DETERMINES PARAMETER OF CONFIG SPACE:
        
        img.obsSpaceGen(robotSize, data, scaleFactor, debug=False)
        
        robotSize = (length in pixels, width in pixels)
        data = camera view (DO NOT ALTER)
        scaleFactor = Downsizing of resoltion i.e 4 means 16 pixels become 1 pixel
        debug = True spits out config slices, False supresses slices
    """
    robotsize = [24,48]
    scaleFactor = 6
    debug_val = True
    
    #initialize RRT

    """
        THIS SECTION DETERMINES PARAMETER OF PLANNER
        r = RRT.rrt(N = 10000,obstacles = obs.T, obstacletype = 'array', maxcoords = obs[0].shape,
            origin = start+[0,'',0],goal = end, live = False, divis = 10)
            
        N = Upper limit on nodes generated in RRT
        obstacles = obstacles as determined by config (DO NOT ALTER)
        obstacletype = format of obstacle data (DO NOT ALTER)
        maxcoords = size of space (DO NOT ALTER)
        origin = Start position of car (DO NOT ALTER)
        goal = Goal, may alter first three values for x,y,theta (DO NOT ALTER REMAINING VALUES)
    """
    N = 10000
    goal = [710,1110, 0]
    live = False
    ##########################################################################################
    


    obs = img.obsSpaceGen(robotSize, data, scaleFactor, debug=debug_val)
    plt.imshow(data,interpolation='nearest') #show 2D representation of map
    r = RRT.rrt(N = N,obstacles = obs.T, obstacletype = 'array', maxcoords = obs[0].shape,
            origin = start+[0,'',0],goal = goal, live = live, divis = 10)
            
    #Perform RRT
    trajectory = r.rrt(dynamics,plotting=True)

    return trajectory

#(verbose = True, plotting=True, dynamics)