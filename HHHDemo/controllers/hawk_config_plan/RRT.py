# All my imports
import math
import random
import sys
import matplotlib.pyplot as plt
import numpy as np

# The RRT class
class rrt():
    def __init__(self, origin = [250, 0, 0, 0,'',0], maxcoords = [500,500], stepsize = 5, N = 15000, obstacles = [[0, 0, 100, 500], [400, 0, 500, 500], [200, 300, 400, 325],
                     [100, 350, 250, 375]], goal = [140, 400, 150, 410], obstacletype = 'vertex', live = False, divis = 1):
        self.origin = origin  # Origin point, in form x,y,parent
        self.maxcoords = maxcoords  # Max values of field. x,y form. Assumes bottom left is 0,0
        self.N = N  # Iterations to run
        self.obstacles = obstacles  # Configuration space, including obstacles. In the form of an array with indeces representing x and y coordinates
        # scaled up by the scale variable, in order to make the system sufficiently continuous. Rectangles only, in form xmin, ymin, xmax, ymax
        self.goal = goal  # goal. Rectangles only, in form xmin, ymin, xmax, ymax
        self.nodesList = [origin]  # list of all nodes

        self.obstacletype = obstacletype    #Whether we're using the vertex obstacle type (manual entry) or the array type (import from obstacleFinder)
        self.live = live    #Whether we're using the "live" plotter or the end-time plotter
        self.divis = divis #draw every divis changes

        self.sweetener = 25 #determines how often to push the tree towards the goal state. EG a value of 100 means it sets the random point to the goal
        #every 100 iterations
        if self.live:   #Turns on interactive plotting if in live mode
            plt.ion()

    # returns the euclidian distance between two nodes
    def eucldist(self,node1, node2):  
        dist = math.sqrt(pow(node1[0] - node2[0], 2) + pow(node1[1] - node2[1], 2))
        return dist

    # Returns the "dist", which is equal to the amount of time it takes to get to that node
    def finddist(self,node1, node2, typec):
        coe = 1
        if typec == 'HOUND':
            Vmax = .5 *210#Max distance for one second movement
            delta_max = 3.039003906 #max radians for one second rotation
            theta_path = math.atan2(node2[0] - node1[0],node2[1] - node1[1])
            theta_diff1 = theta_path - node1[2]
            if theta_diff1 > math.pi:
                theta_diff1 = theta_diff1 - 2*math.pi
            elif theta_diff1 <= -math.pi:
                theta_diff1 = theta_diff1 + 2*math.pi
                
            if node1[1] == node2[1] and node1[0] == node2[0]:
                theta_path = node2[2]
                
            theta_diff2 = node2[2] - theta_path
            if theta_diff2 > math.pi:
                theta_diff2 = theta_diff2 - 2*math.pi
            elif theta_diff2 <= -math.pi:
                theta_diff2 = theta_diff2 + 2*math.pi
        else:
            coe = .5 # This coefficient reduces the weight of turns cause they take a long time
            Vmax = .4*210 #Max distance for one second movement
            delta_max = 0.596807115 #max radians for one second rotation
            theta_diff1 = 0 # Hippo has no theta one to cover so its zero
            theta_diff2 = node2[2] - node1[2]
            if theta_diff2 > math.pi:
                theta_diff2 = theta_diff2 - 2*math.pi
            elif theta_diff2 <= -math.pi:
                theta_diff2 = theta_diff2 + 2*math.pi

        return (self.eucldist(node1, node2) / Vmax) + coe*(abs(theta_diff1/delta_max) + abs(theta_diff2/delta_max))


    # generates a r  coords[0]), random.uniform(0, self.maxcoords[1]),random.uniform(0,2*math.pi),0,'',0]
    def randomPoint(self):  
        point = [random.uniform(0, self.maxcoords[0]), random.uniform(0, self.maxcoords[1]),random.uniform(-math.pi,math.pi),0,'',0]
        return point

    # checks if the point is inside an obstacle. if it is, returns the origin. ##FUTURE## return the closest allowed point
    def obsCheck(self, point, obstacles):  
        if self.obstacletype == 'vertex':
            for o in obstacles:
                if (((o[0] < point[0] + self.robotRadius < o[2]) or (o[0] < point[0] - self.robotRadius < o[2])) and (
                        (o[1] < point[1] + self.robotRadius < o[3]) or (o[1] < point[1] - self.robotRadius < o[3]))):
                    return True
        elif self.obstacletype == 'array':  # array type obstacles not currently using robot radius
            xflr = math.floor(
                point[0]) - 1  # create floor and ceilinged variables to make sure we cover all possible cases.
            yflr = math.floor(
                point[1]) - 1  # This is because we need to reference the obstacles array, which needs discrete indeces.
            xcl = math.ceil(point[0]) - 1
            ycl = math.ceil(point[1]) - 1
            xmax = obstacles.shape[0] - 1
            ymax = obstacles.shape[1] - 1

            theta = point[2] * 180 / (2 * math.pi)
            theta = round(theta / 5)
            theta = theta % 36

            if xflr >= xmax or xcl >= xmax:  # make sure bounds are not violated
                xflr = xcl = xmax
            if yflr >= ymax or ycl >= ymax:
                yflr = ycl = ymax

            if (obstacles[xflr][yflr][theta] or obstacles[xflr][ycl][theta] or obstacles[xcl][yflr][theta] or
                    obstacles[xcl][ycl][
                        theta]):  # if the rounded location (via any rounding scheme) is a wall (True in the obstacle array), say so
                return True
        return False

    # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
    def takestepHOUND(self,startnode, targetnode, nodes):  
        dt = 0.008        #Timestep in seconds
        endtime = 2.4     #endtime in seconds
        instructionVector = []  #vector for instructions
        # Full spin and Full velocity
        theta_dot = 3.039003906
        V = .5*210
        currentPos = [startnode[0],startnode[1],startnode[2],self.nodesList.index(startnode),'',0]  #initialize currentPosition to startPosition
        prevPos = currentPos    #initialize prevPos to currentPos
        theta_path = math.atan2(targetnode[0] - startnode[0], targetnode[1] - startnode[1]) #calculate theta_path for the given two points

        xvel = V*math.cos(theta_path)   #calculate X and Z components of velocity
        zvel = V*math.sin(theta_path)
        
        # Hound Movement come in three stages, rotate, move, rotate
        stage1 = True
        stage2 = False
        stage3 = False
        
        # Check if some stages can be bypassed
        if abs(currentPos[2] - theta_path) < theta_dot * dt or abs(currentPos[2] + math.pi - theta_path) < theta_dot * dt or abs(currentPos[2] - math.pi - theta_path) < theta_dot * dt:
            stage1 = False
            stage2 = True
            if self.eucldist(currentPos,targetnode) < V*dt:
                stage2 = False
                stage3 = True
              
        # Counter keeps track of number of timesteps for a given input  
        counter = 0
        
        # Theta diff is to find out whether you should turn left or right to align theta for the path
        theta_diff = theta_path - startnode[2]
        if theta_diff > math.pi:
            theta_diff = theta_diff - 2*math.pi
        elif theta_diff <= -math.pi:
            theta_diff = theta_diff + 2*math.pi
        path_right = True
        if (theta_diff < 0):
            path_right = False    
        
        # Final diff is to find out whether you should turn left or right to align theta for the final spot
        final_diff = targetnode[2] - theta_path
        if final_diff > math.pi:
            final_diff = final_diff - 2*math.pi
        elif final_diff <= -math.pi:
            final_diff = final_diff + 2*math.pi 
        final_right = True
        if (final_diff < 0):
            final_right = False

        # Incremental step
        for t in range(0,int(endtime/dt)):  #for each time step
            if self.obsCheck(currentPos, self.obstacles):   #catch if currently in an obstacle
                return prevPos      #Invalidate entire path and return start node
                
            # Stage one aligns theta for path
            # If theta is aligned for path, append instruction and reset variables
            # Else take a step
            if stage1:
                if abs(currentPos[2] - theta_path) < theta_dot * dt:
                    currentPos[2] = theta_path
                    stage2 = True
                    stage1 = False
                    t = t - 1 # This was a decision and not a step so draw t back by one
                    if path_right:
                        instructionVector.append([4, -4, counter])
                    else:
                        instructionVector.append([-4, 4, counter])
                    counter = 0
                    instructionVector.append([0, 0, 25])
                else:
                    counter = counter + 1
                    if path_right:
                        currentPos[2] = currentPos[2] + theta_dot * dt
                    else:
                        currentPos[2] = currentPos[2] - theta_dot * dt
            
            # Stage two moves along path           
            elif stage2:
                if self.eucldist(currentPos,targetnode) < V*dt:
                    currentPos[0] = targetnode[0]
                    currentPos[1] = targetnode[1]
                    stage3 = True
                    stage2 = False
                    t = t - 1
                    instructionVector.append([20, 20, counter])
                    counter = 0
                    instructionVector.append([0, 0, 15])
                else:
                    counter = counter + 1
                    currentPos[1] = currentPos[1] + xvel * dt
                    currentPos[0] = currentPos[0] + zvel * dt
               
            # Stage three aligns theta for final spot     
            elif stage3:
                if abs(currentPos[2]-targetnode[2]) < theta_dot*dt:
                    currentPos[2] = targetnode[2]
                    break
                else:
                    counter = counter + 1
                    if final_right:
                        currentPos[2] = currentPos[2] + theta_dot * dt
                    else:
                        currentPos[2] = currentPos[2] - theta_dot * dt
        
        # Gotta check that last instruction                
        if self.obsCheck(currentPos, self.obstacles):    #catch if currently in an obstacle
            return prevPos                               #Invalidate all if not good
        
        # This if ladder catches final instructions since loop will terminate mid instruction
        if stage3:
            if final_right:
                instructionVector.append([4, -4, counter])
            else:
                instructionVector.append([-4, 4, counter])
            instructionVector.append([0, 0, 25])
        elif stage2:
            instructionVector.append([20, 20, counter])
            instructionVector.append([0, 0, 15])
        elif stage1:
            if path_right:
                instructionVector.append([4, -4, counter])
            else:
                instructionVector.append([-4, 4, counter])
            instructionVector.append([0, 0, 25])

        # Setup for return and return
        currentPos[4] = instructionVector
        return currentPos

    # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
    def takestepHIPPO(self,startnode, targetnode, nodes):  
        dt = 0.008        #Timestep in seconds
        endtime = 1.6     #endtime in seconds
        instructionVector = []  #vector for instructions
        # Full spin and Full velocity
        theta_dot = 0.596807115
        V = .4*210
        currentPos = [startnode[0],startnode[1],startnode[2],self.nodesList.index(startnode),'',0]  #initialize currentPosition to startPosition
        prevPos = currentPos    #initialize prevPos to currentPos
        theta_path = math.atan2(targetnode[0] - startnode[0], targetnode[1] - startnode[1]) #calculate theta_path for the given two points

        xvel = V*math.cos(theta_path)   #calculate X and Z components of velocity
        zvel = V*math.sin(theta_path)
        
        # Hippo Movement come in two stages move then rotate
        stage1 = True
        stage2 = False
        
        # Check if some stages can be bypassed
        if self.eucldist(currentPos,targetnode) < V*dt:
            stage1 = False
            stage2 = True
              
        # Counter keeps track of number of timesteps for a given input  
        counter = 0
        
        # Theta diff is to find out inputs for wheels
        theta_diff = theta_path - startnode[2]
        if theta_diff > math.pi:
            theta_diff = theta_diff - 2*math.pi
        elif theta_diff <= -math.pi:
            theta_diff = theta_diff + 2*math.pi
        Vx = V*math.cos(theta_diff)/210
        Vz = V*math.sin(theta_diff)/210
        wheels14 = (Vx - Vz)/.05
        wheels23 = (Vx + Vz)/.05   
        
        # Final diff is to find out whether you should turn left or right to align theta for the final spot
        final_diff = targetnode[2] - startnode[2]
        if final_diff > math.pi:
            final_diff = final_diff - 2*math.pi
        elif final_diff <= -math.pi:
            final_diff = final_diff + 2*math.pi 
        final_right = True
        if (final_diff < 0):
            final_right = False

        # Incremental step
        for t in range(0,int(endtime/dt)):  #for each time step
            if self.obsCheck(currentPos, self.obstacles):   #catch if currently in an obstacle
                return prevPos      #Invalidate entire path and return start node
            
            # Stage two moves along path           
            if stage1:
                if self.eucldist(currentPos,targetnode) < V*dt:
                    currentPos[0] = targetnode[0]
                    currentPos[1] = targetnode[1]
                    stage2 = True
                    stage1 = False
                    t = t - 1
                    instructionVector.append([wheels14, wheels23, wheels23, wheels14, counter])
                    counter = 0
                    instructionVector.append([0, 0, 0, 0, 5])
                else:
                    counter = counter + 1
                    currentPos[0] = currentPos[0] + zvel * dt
                    currentPos[1] = currentPos[1] + xvel * dt
               
            # Stage three aligns theta for final spot     
            elif stage2:
                if abs(currentPos[2]-targetnode[2]) < theta_dot*dt:
                    currentPos[2] = targetnode[2]
                    break
                else:
                    counter = counter + 1
                    if final_right:
                        currentPos[2] = currentPos[2] + theta_dot * dt
                    else:
                        currentPos[2] = currentPos[2] - theta_dot * dt
        
        # Gotta check that last instruction                
        if self.obsCheck(currentPos, self.obstacles):    #catch if currently in an obstacle
            return prevPos                               #Invalidate all if not good
        
        # This if ladder catches final instructions since loop will terminate mid instruction
        if stage2:
            if final_right:
                instructionVector.append([-14, 14, -14, 14, counter])
            else:
                instructionVector.append([14, -14, 14, -14, counter])
            instructionVector.append([0, 0, 0, 0, 5])
        elif stage1:
            instructionVector.append([wheels14, wheels23, wheels23, wheels14, counter])
            instructionVector.append([0, 0, 0, 0, 5])

        # Setup for return and return
        currentPos[4] = instructionVector
        return currentPos
    # finds the closest node to newnode in nodelist nodes
    def findclosest(self,nodes, newnode, typec): 
        distances = []
        for i in nodes:
            distances.append(self.finddist(i,newnode, typec))
        return nodes[distances.index(min(distances))]

    # checks if the point is within the goal. if not, it sets goalfound to false. if it is, it returns the node path it took and sets goalfound to false
    def checkgoal(self,nodelist, node, goal):  
        goalpath = []
        tracenode = []

        if (node[0] == goal[0] and node[1] == goal[1] and node[2] == goal[2]): #if the node is in the correct range
            goalfound = True        #set goal flag
            tracenode = node        #stores "winning" node in tracenode
            goalpath.append(tracenode)  #adds tracenode to the goal trajectory
            while (tracenode[3] != 0):  #while we havent yet hit the origin
                tracetemp = nodelist[tracenode[3]]  #get the parent node of tracenode
                goalpath.append(tracetemp)      #append parent to trajectory
                tracenode = tracetemp       #set the parent node to the current node and repeat
            goalpath.append(self.origin)  # since the loop breaks once we hit the origin, add the origin to the end
        else:
            goalfound = False

        return [goalfound, goalpath]

    # initializes plot by drawing obstacles, goal, and origin as well as setting the axes
    def initplot(self, goal, obstacles):  
        if self.obstacletype == 'vertex':
            for o in obstacles:
                obsbox = [[o[0], o[2], o[2], o[0], o[0]], [o[1], o[1], o[3], o[3], o[1]]]
                plt.plot(obsbox[0], obsbox[1], 'r')
        plt.scatter(goal[0], goal[1],s=25, c='g')
        plt.scatter(self.origin[0], self.origin[1], s=25, c='r')
        plt.xlim(0, self.maxcoords[0])
        plt.ylim(0, self.maxcoords[1])

    #draws connecting lines between parent and child nodes
    def drawparentlines(self, nodelist):    
        filterlist = [self.origin]
        #print(self.origin)
        for n in nodelist:
            if n[3]==-1:
                pass
            else:
                filterlist.append(n)
        for node in filterlist:             #plot each "jump" from child node to parent node
            if(node == nodelist[node[3]]):
                pass
            else:
                jumplist = [node, nodelist[node[3]]]
                plt.plot([jumplist[0][0],jumplist[1][0]],[jumplist[0][1],jumplist[1][1]],'b')
                if self.live and nodelist.index(node) % self.divis == 0:   #Draw these consecutively if in live mode
                    plt.draw()
                    plt.pause(0.0001)
    # Yes
    def getOrigin(self):
        return self.origin
        
    #Main implementation of RRT
    def rrt(self,dynamics,verbose = False, plotting = False):   
        xg=[]
        yg=[]
        self.initplot(self.goal, self.obstacles)    #initialize plot

        for k in range(0, self.N):      #create (or attempt to create) N nodes
            if k % self.sweetener != 0 and k < self.N - 20:
                xrand = self.randomPoint()  #choose a random point
            else:
                xrand = self.goal
            xnear = self.findclosest(self.nodesList, xrand, dynamics)     #find the nearest node to the random point
            #xnear = self.findclosestOPT(self.nodesList, xrand)
            if dynamics == 'HOUND':
                xnew = self.takestepHOUND(xnear, xrand, self.nodesList)  #take one step towards the random point from the nearest node and create a new node
            else:
                xnew = self.takestepHIPPO(xnear, xrand,
                                          self.nodesList)  # take one step towards the random point from the nearest node and create a new node
            #xnew = self.optimize(self.nodesList,xnew)
            [goalbool, goalpath] = self.checkgoal(self.nodesList, xnew, self.goal)  #check the new node to see if it's in the goal zone
            if (goalbool):
                [xg, yg, tg, zg, sg,cg] = list(zip(*goalpath))     #If it does succeed, stop creating new nodes and return the goal path
                break
            else:   #otherwise, just add it to the list and continue
                self.nodesList.append(xnew)
            if verbose: #debug info, only dump if verbose
                print(k)
        if plotting == True:# Plot if that's enabled

            self.drawparentlines(self.nodesList)

            if goalbool:
                plt.plot(xg, yg, 'y')
            if self.live:   #If live, draw the goal to the live graph
                 plt.draw()
                 plt.pause(0.01)
                 plt.ioff()
            plt.show()

        xg = (np.array(xg)).tolist()   #scale trajectory down from increased scale
        yg = (np.array(yg)).tolist()
        trajectory = []
        for i in range(0,len(xg)):
            trajectory.append([xg[i],yg[i],tg[i],sg[i]])

        return trajectory[::-1] #return the trajectory to the goal (reverse it, its in goal -> origin order until this line