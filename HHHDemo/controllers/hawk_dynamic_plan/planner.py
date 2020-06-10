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
    goal = [710,1110, 0]
    ##########################################################################################
    

    #initialize RRT
    obs = img.obsSpaceGen(robotSize, data, scaleFactor = 6, debug=False)
    plt.imshow(data,interpolation='nearest') #show 2D representation of map
    r = RRT.rrt(N = 6000,obstacles = obs.T, obstacletype = 'array', maxcoords = obs[0].shape,
            origin = start+[0,'',0],goal = goal, live = False, divis = 10)
            
    #Perform RRT
    trajectory = r.rrt(dynamics,plotting=True)

    return trajectory

#(verbose = True, plotting=True, dynamics)