import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np


class dynObsPlanner():

    def __init__(self, obs, dynObs, divis=1,N=1000):
        self.maxcoords = obs.shape  # Max values of field. x,y form. Assumes bottom left is 0,0
        self.N = N  # Iterations to run
        self.obstacles = np.transpose(np.add(obs,dynObs))[:, ::-1]  # Configuration space, including obstacles. In the form of an array with indeces representing x and y coordinates
        self.divis = divis  # draw every divis changes
        self.sweetener = 25  # determines how often to push the tree towards the goal state. EG a value of 100 means it sets the random point to the goal

    def eucldist(self, node1, node2):  # returns the euclidian distance between two nodes
        dist = math.sqrt(pow(node1[0] - node2[0], 2) + pow(node1[1] - node2[1], 2))
        return dist


    def randomPoint(self):  # generates a r  coords[0]), random.uniform(0, self.maxcoords[1]),random.uniform(0,2*math.pi),0,'',0]
        point = [random.uniform(0, self.maxcoords[0]), random.uniform(0, self.maxcoords[1]), 0, 0]
        return point

    def obsCheck(self, point, obstacles):  # checks if the point is inside an obstacle. if it is, returns the origin. ##FUTURE## return the closest allowed point
            xflr = math.floor(
                point[0]) - 1  # create floor and ceilinged variables to make sure we cover all possible cases.
            yflr = math.floor(
                point[1]) - 1  # This is because we need to reference the obstacles array, which needs discrete indeces.
            xcl = math.ceil(point[0]) - 1
            ycl = math.ceil(point[1]) - 1
            xmax = obstacles.shape[0] - 1
            ymax = obstacles.shape[1] - 1

            if xflr >= xmax or xcl >= xmax:  # make sure bounds are not violated
                xflr = xcl = xmax
            if yflr >= ymax or ycl >= ymax:
                yflr = ycl = ymax

            if (obstacles[xflr][yflr]==1 or obstacles[xflr][ycl]==1 or obstacles[xcl][yflr]==1 or obstacles[xcl][ycl]==1):  # if the rounded location (via any rounding scheme) is a wall (True in the obstacle array), say so
                return 1
            elif (obstacles[xflr][yflr]==2 or obstacles[xflr][ycl]==2 or obstacles[xcl][yflr]==2 or obstacles[xcl][ycl]==2):  # if the rounded location (via any rounding scheme) is a wall (True in the obstacle array), say so
                return 2
            elif (obstacles[xflr][yflr]==3 or obstacles[xflr][ycl]==3 or obstacles[xcl][yflr]==3 or obstacles[xcl][ycl]==3):  # if the rounded location (via any rounding scheme) is a wall (True in the obstacle array), say so
                return 3
            return False

    def takestep(self, startnode, targetnode,nodes):  # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
        dt = 0.008  # Timestep in seconds
        endtime = 1.6  # endtime in seconds
        dynO = 0  # vector for instructions
        # Full spin and Full velocity
        V = .25 * 210
        currentPos = [startnode[0], startnode[1], nodes.index(startnode),0]  # initialize currentPosition to startPosition
        prevPos = currentPos  # initialize prevPos to currentPos
        # Counter keeps track of number of timesteps for a given input
        counter = 0
        theta_path = math.atan2(targetnode[0] - startnode[0],
                                targetnode[1] - startnode[1])  # calculate theta_path for the given two points

        xvel = V * math.cos(theta_path)  # calculate X and Z components of velocity
        zvel = V * math.sin(theta_path)
        # Incremental step
        for t in range(0, int(endtime / dt)):  # for each time step
            if self.obsCheck(currentPos, self.obstacles)==1:  # catch if currently in an obstacle
                return prevPos  # Invalidate entire path and return start node

            if self.eucldist(currentPos, targetnode) < V * dt:
                currentPos[0] = targetnode[0]
                currentPos[1] = targetnode[1]
                t = t - 1
            else:
                currentPos[1] = currentPos[1] + xvel * dt
                currentPos[0] = currentPos[0] + zvel * dt

        # Gotta check that last instruction
        if self.obsCheck(currentPos, self.obstacles)==1:  # catch if currently in an obstacle
            return prevPos  # Invalidate all if not good
        elif self.obsCheck(currentPos, self.obstacles)==2:  # catch if currently in an obstacle
            dynO=2
        elif self.obsCheck(currentPos, self.obstacles)==3:  # catch if currently in an obstacle
            dynO=3
        # This if ladder catches final instructions since loop will terminate mid instruction
        # Setup for return and return

        currentPos[3] = dynO
        return currentPos


    def findclosest(self, nodes, newnode):  # finds the closest node to newnode in nodelist nodes
        distances = []
        for i in nodes:
            distances.append(self.eucldist(i, newnode))
        return nodes[distances.index(min(distances))]

    def checkgoal(self, nodelist, node,
                  goal,origin):  # checks if the point is within the goal. if not, it sets goalfound to false. if it is, it returns the node path it took and sets goalfound to false
        goalpath = []
        tracenode = []

        if (node[0] == goal[0] and node[1] == goal[1]):  # if the node is in the correct range
            goalfound = True  # set goal flag
            tracenode = node  # stores "winning" node in tracenode
            goalpath.append(tracenode)  # adds tracenode to the goal trajectory
            while (tracenode[2] != 0):  # while we havent yet hit the origin
                tracetemp = nodelist[tracenode[2]]  # get the parent node of tracenode
                goalpath.append(tracetemp)  # append parent to trajectory
                tracenode = tracetemp  # set the parent node to the current node and repeat
            goalpath.append(origin)  # since the loop breaks once we hit the origin, add the origin to the end
        else:
            goalfound = False

        return [goalfound, goalpath]

    def initplot(self, goal,
                 obstacles,origin):  # initializes plot by drawing obstacles, goal, and origin as well as setting the axes
        plt.imshow(obstacles.T, interpolation='nearest')  # show 2D representation of map
        plt.scatter(goal[0], goal[1], s=25, c='g')
        plt.scatter(origin[0], origin[1], s=25, c='r')
        plt.xlim(0, self.maxcoords[0])
        plt.ylim(0, self.maxcoords[1])

    def drawparentlines(self, nodelist,origin):  # draws connecting lines between parent and child nodes
        filterlist = [origin]

        for n in nodelist:
            if n[2] == 0:
                pass
            else:
                filterlist.append(n)
        for node in filterlist:  # plot each "jump" from child node to parent node
            if (node == nodelist[node[2]]):
                pass
            else:

                jumplist = [node, nodelist[node[2]]]
                plt.plot([jumplist[0][0], jumplist[1][0]], [jumplist[0][1], jumplist[1][1]], 'b')



    def dyncheck(self,start,goal, verbose=False, plotting=True):  # Main implementation of RRT
        origin = start + [0,0]
        xg = []
        yg = []
        nodesList = [origin]
        cg = []
        self.initplot(goal, self.obstacles,origin)  # initialize plot

        for k in range(0, self.N):  # create (or attempt to create) N nodes
            if k % self.sweetener != 0:
                xrand = self.randomPoint()  # choose a random point
            else:
                xrand = goal
            xnear = self.findclosest(nodesList, xrand)  # find the nearest node to the random point
            xnew = self.takestep(xnear, xrand, nodesList)  # take one step towards the random point from the nearest node and create a new node
            [goalbool, goalpath] = self.checkgoal(nodesList, xnew, goal,origin)  # check the new node to see if it's in the goal zone
            if (goalbool):
                [xg, yg, zg, cg] = list(
                    zip(*goalpath))  # If it does succeed, stop creating new nodes and return the goal path
                break
            else:  # otherwise, just add it to the list and continue
                nodesList.append(xnew)
            if verbose:  # debug info, only dump if verbose
                print(k)
        if plotting:  # Plot if that's enabled
            self.drawparentlines(nodesList,origin)
            if goalbool:
                plt.plot(xg, yg, 'y')
            plt.show()

        trajectory = cg

        return trajectory[
               ::-1]  # return the trajectory to the goal (reverse it, its in goal -> origin order until this line