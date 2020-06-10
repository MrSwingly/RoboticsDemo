"""node_marker controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot, Emitter, Receiver, Motor, Keyboard
from matplotlib import pyplot as plt
import numpy as np
import threading
import collision_geometry as cg
import random
import time

# create the Robot instance.
robot = Robot()
random.seed()

communication_channel = 1
# get the time step of the current world.
timestep = 32
file = open("my_file.txt","w")
emit = robot.getEmitter("emitter")
emit.setChannel(communication_channel)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
hound_radius = 0.6 #real radius in meter
hippo_radius = 0.6 #real radius in meter
scale_const = 1.5
step = 8

def path_scaler(path, scale_const):
	new_path = []
	for i in path:
		new_node = []
		for j in i:
			new_node.append(j*scale_const)
		new_path.append(new_node)
	return new_path

def plug_parents_node(ori_hound):
	new_hound = []
	for i in range(len(ori_hound)):
		node = []
		if i == 0:
			node = ori_hound[i] + ori_hound[i]
		else:
			node = ori_hound[i] + ori_hound[i-1]
		new_hound.append(node)
	return new_hound

def generate_random_trajectory(n):
	traj = []
	sum = 0
	for i in range(n):
		sum = 0
		loopo = random.randint(0,10)
		for j in range(loopo):
			sum = sum + random.random()
		traj.append([sum])
	for i in range(n):
		sum = 0
		loopo = random.randint(0,10)
		for j in range(loopo):
			sum = sum + random.random()
		traj[i].append(sum)
	return traj

def chose_path(n):
	if n == 0:
		return [[0.5,0.5], [0.6,0.7], [1.2,1.3],[1.4,2], [1.5,1.6], [2,2.4], [3.6,2], [3.8,2.5], [4,2.7],[4.3,2.8],[4.4,3]], [[2.5,0],[2.5,1],[2.6,1.2],[2.8,1.4],[3.1,1.8],[3.2,3.4],[3.4,3.5],[3.5,4],[4,3.4],[5,4]]
	if n == 2:
		return [[0.5,0.5], [0.6,0.7], [3,0],[1,3,4],[4,5],[2,4],[5,5]], [[2.5,0],[2.5,1],[1,1.5], [1.4,1.5], [1,1.6], [2,3], [3,4],[3.5,4.5],[1,2]]
	else:
		return [[0.5,0.5], [0.6,0.7],[1.333,1.333],[2.2,2.66], [4.5,2.4], [0,1],[3,5],[4,6]], [[2.5,0],[2.5,1],[4,1], [4.5,2.5], [2,2.66], [4,6.5], [4.5,4], [6,5], [5,1], [3,2]]

def RRT():
	n = random.randint(0,3)
	hippo_path, hound_path = chose_path(n)
	#sleep 2 secs before generate another random trajectory
	#hippo_path = [[1.333,1.333],[2.2,2.66], [4.5,2.4], [0,1]]
	#hound_path = [[4,1], [4.5,2.5], [2,2.66], [4,6.5], [4.5,4], [6,5], [5,1], [3,2]]
	#hippo_path = [[3,0],[1,3,4],[4,5],[2,4],[5,5]]
	#hound_path = [[1,1.5], [1.4,1.5], [1,1.6], [2,3], [3,4],[3.5,4.5],[1,2]]
	#hound_path, hippo_path = plug_parents_node(hound_path), plug_parents_node(hippo_path)
	hippo_path = path_scaler(hippo_path,scale_const)
	hound_path = path_scaler(hound_path,scale_const)
	return hound_path, hippo_path

s = "0.5 0.5 0.5 0.5 a 0 0 0 0 b 0 0 0 0 c 0 0 0 0 n 0.5 0.5 0.5 0.5 a 0 0 0 0 b 0 0 0 0 c 0 0 0 0 n e"

def conv_trajectory2strformat(traj):
    message = ""
    for i in traj:
        message = message + str(i[0]) + ' ' + str(i[1]) + ' ' + 'n '
    message = message + 'e'
    return message

def convert_to_binary(string_):
    return bytes(string_,'utf-8')

def send_trajectory2hound_hippo(hound_path,hippo_path):
	str_format_hound_path = conv_trajectory2strformat(hound_path)
	str_format_hound_path = '22 ' + str_format_hound_path
	str_format_hippo_path = conv_trajectory2strformat(hippo_path)
	str_format_hippo_path = '11 ' + str_format_hippo_path
	print(str_format_hippo_path)
	bin_data_hound = convert_to_binary(str_format_hound_path)
	bin_data_hippo = convert_to_binary(str_format_hippo_path)
	emit.send(bin_data_hound)
	emit.send(bin_data_hippo)

#================main()==================#

send_once = True

#class Thread_example:
    #def __init__(self):
	#def hello_1(self):
    #           #lock.acquire()
    #            print("hello1\n")
    #            #self.count = self.count + 1
     #           #lock.release()
     #           print("hello1\n")
      #      def hello_2(self):
                #lock.acquire()
       #         print("hello2\n")
                #self.count = self.count + 1
                #lock.release()
        #        print("hello2\n")

#	def webots_hack(self):
#		while robot.step(timestep) != -1:
#			pass
		
#	def execute_main(self):
#		global send_once

#This a function to handler type 2 request from other robot. It replies back information neccessary for the sender robot to resolve deadlock
#Reply in a form of "2 my_id current_state current_node.x current_node.y next_node.x next_node.y, type A, type B, type C". This function return list

fig, ax = plt.subplots()
line, = ax.plot([], [], lw=3)
ax.grid()
theta = np.linspace(0,2*np.pi,100)
ax.set_ylim(-1,15)
ax.set_xlim(0,15)

def circle(cx,cy,r):
    x = r * np.cos(theta) + cx
    y = r * np.sin(theta) + cy
    return x,y

#line formed by two points from p1 to p2
def line(p1,p2):
    if p2.x - p1.x == 0:
        quality = int(abs(p2.y - p1.y) * 50)
		
        y = np.linspace(float(p1.y),float(p2.y),quality)
        r = y.shape
        x = np.full((r[0],1),p1.x)
        return x,y
    if p2.y - p1.y == 0:
        quality = int(abs(p2.x - p1.x) * 50)
        x = np.linspace(float(p1.x),float(p2.x),quality)
        r = x.shape
        y = np.full((r[0],1),p1.y)
        return x,y
    m = (p2.y - p1.y)/(p2.x-p1.x)
    b = p1.y  - m*p1.x
    quality = int(abs(p1.x - p2.x) * 50)
    x = np.linspace(float(p1.x),float(p2.x),quality)
    y = m * x + b
    return x,y


def plt_my_path(path,radius,color):
    for i in range(len(path) - 1):
        cx, cy = circle(path[i][0],path[i][1],radius)
        lx, ly = line(cg.Point(path[i][0],path[i][1]),cg.Point(path[i+1][0],path[i+1][1]))
        ax.plot(cx,cy,color = color)
        ax.plot(lx,ly,color = color)

    gx, gy = circle(path[len(path)-1][0],path[len(path)-1][1],radius)
    ax.plot(gx,gy,color = color)


#print("press 1 to 3 to choose the path\n")
#input_char = msvcrt.getch()
#print("my input",input_char)

while robot.step(timestep) != -1:
	#print("my key", key)
	if send_once:
		hound_path, hippo_path = RRT()
		hound_for_plt, hippo_for_plt = hound_path, hippo_path
		plt_my_path(hound_for_plt,hound_radius,"g")
		plt_my_path(hippo_for_plt,hippo_radius,"r")
		plt.show()
		send_trajectory2hound_hippo(hound_path,hippo_path)
		send_once = False #only send one time
			





#if hound_for_plt[1][0] - hound_for_plt[0][0] == 0:
#    anim_quality = abs(hound_for_plt[0][1] - hound_for_plt[1][1]) * 50
#elif hound_for_plt[1][1] - hound_for_plt[0][1] == 0:
#    anim_quality = abs(hound_for_plt[0][0] - hound_for_plt[1][0]) * 50
#else:
#    anim_quality = abs(hound_for_plt[0][0] - hound_for_plt[1][0])*50

#index_i = 0

#def animate(i):
#    real_i = i * 0.2
#    if i == anim_quality:
#        global  index_i = index_i + 1

#    inc_x = i
#    x, y = circle(hound_for_plt[index_i][0] + ,hound_for_plt[index_i][1] + ,hound_radius)

#anim = FuncAnimation(fig, animate, init_func=init,frames=simulation_step,interval=speed)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.	

