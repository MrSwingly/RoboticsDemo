"""emitter_receiver_py controller."""
import threading
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Emitter, Receiver, Motor, GPS, Compass
import math
import numpy as np
import collision_geometry as cg
        
#========================hippopy_code, code for driving, dont have to read this================================================

#plateform_speed is the speed of the body in m/s for SPEED define as 4.0 bellow
SPEED = 4.0
DISTANCE_TOLERANCE = 0.02
ANGLE_TOLERANCE = 0.02

K1 = 80.0
K2 = 1.0
K3 = 1.0

timestep =  32

#============code for tiny math===========================

class Vector2:
	def __init__(self,u,v):
		self.u = u
		self.v = v

class Vector3:
	def __init__(self,u,v,w):
		self.u = u
		self.v = v
		self.w = w

class Matrix33:
	def __init__(self,a,b,c):
		self.a = a
		self.b = b
		self.c = c

def vector3_set_values(vect, u, v, w):
	vect.u = u
	vect.v = v
	vect.w = w
	

def matrix33_set_values(m,au,av,aw,bu,bv,bw,cu,cv,cw):
	vector3_set_values(m.a,au,av,aw)
	vector3_set_values(m.b,bu,bv,bw)
	vector3_set_values(m.c,cu,cv,cw)


def matrix33_set_identity(m):
	matrix33_set_values(m,1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


def matrix33_mult_vector3(res, m, v):
	res.u = m.a.u * v.u + m.b.u * v.v + m.c.u * v.w
	res.v = m.a.v * v.u + m.b.v * v.v + m.c.v * v.w
	res.w = m.a.w * v.u + m.b.w * v.v + m.c.w * v.w

def vector2_norm(v):
	return math.sqrt(v.u * v.u + v.v * v.v)


def vector2_minus(v,v1,v2):
	v.u = v1.u - v2.u
	v.v = v1.v - v2.v

def vector2_angle(v1,v2):
	return math.atan2(v2.v, v2.u) - math.atan2(v1.v, v1.u)

def bound(v,a,b):
	if v > b:
		return b
	elif v < a:
		return a
	else:
		return v

#==========================end_tiny math code====================================

lock = 0
node_num = 0

class goto_struct:
	def __init__(self,v,a,r):
		self.v_target = v
		self.alpha = a
		self.reached = r

class node:
	def __init__(self,x,z,alpha):
		self.x = x
		self.z = z
		self.alpha = alpha


init_node = node(0,0,0)
init_lock = 0
robot = Robot()

motors = []
motors.append(robot.getMotor("wheel1"))
motors.append(robot.getMotor("wheel2"))
motors.append(robot.getMotor("wheel3"))
motors.append(robot.getMotor("wheel4"))

gps = robot.getGPS("gps")
compass = robot.getCompass("compass")
gps.enable(timestep)
compass.enable(timestep)
goto_data = goto_struct(Vector2(0.0,0.0),0.0,False)

def base_set_wheel_speeds_helper(speeds):
	for i in range(len(motors)):
		motors[i].setPosition(float('+inf'))
		motors[i].setVelocity(speeds[i])

def base_goto_set_target(x,z,alpha):
	goto_data.v_target.u = x
	goto_data.v_target.v = z
	goto_data.alpha = alpha
	goto_data.reached = False

def base_reset():
	speeds = [0.0, 0.0, 0.0, 0.0]
	base_set_wheel_speeds_helper(speeds)

def base_goto_run():
	v_gps = Vector2(gps.getValues()[0], gps.getValues()[2])
	v_front = Vector2(compass.getValues()[0], compass.getValues()[1])
	v_right = Vector2(-v_front.v, v_front.u)
	v_north = Vector2(1.0,0.0)
	
	v_dir = Vector2(0.0,0.0)
	vector2_minus(v_dir,goto_data.v_target, v_gps)
	distance = vector2_norm(v_dir)
	
	theta = vector2_angle(v_front, v_north)
	delta_angle = theta - goto_data.alpha
	
	transform = Matrix33(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
	matrix33_set_identity(transform)
	transform.a.u = v_front.u;
	transform.a.v = v_right.u;
	transform.b.u = v_front.v;
	transform.b.v = v_right.v;
	transform.c.u = -v_front.u * v_gps.u - v_front.v * v_gps.v;
	transform.c.v = -v_right.u * v_gps.u - v_right.v * v_gps.v;
	
	v_target_tmp = Vector3(goto_data.v_target.u, goto_data.v_target.v, 1.0)
	v_target_rel = Vector3(0,0,0)
	matrix33_mult_vector3(v_target_rel, transform, v_target_tmp)
	
	speeds = [0.0, 0.0, 0.0, 0.0]
	speeds[0] = -delta_angle / np.pi * K1
	speeds[1] = delta_angle / np.pi * K1
	speeds[2] = -delta_angle / np.pi * K1
	speeds[3] = delta_angle / np.pi * K1
	
	speeds[0] = speeds[0] + v_target_rel.u * K2
	speeds[1] = speeds[1] + v_target_rel.u * K2
	speeds[2] = speeds[2] + v_target_rel.u * K2
	speeds[3] = speeds[3] + v_target_rel.u * K2
	
	speeds[0] = speeds[0] - v_target_rel.v * K3
	speeds[1] = speeds[1] + v_target_rel.v * K3
	speeds[2] = speeds[2] + v_target_rel.v * K3
	speeds[3] = speeds[3] - v_target_rel.v * K3

	for i in range(4):
		speeds[i] = speeds[i] / (K1 + K2 + K3)
		speeds[i] = speeds[i] * SPEED
		speeds[i] = speeds[i] * 200.0
		speeds[i] = bound(speeds[i], -SPEED, SPEED)
		
	base_set_wheel_speeds_helper(speeds)
	
	#if distance < DISTANCE_TOLERANCE and delta_angle < ANGLE_TOLERANCE and delta_angle > -ANGLE_TOLERANCE:
	#	goto_data.reached = True
	#("distance: ", distance)
	if distance < DISTANCE_TOLERANCE:
		goto_data.reached = True
	

def base_goto_reached():
	return goto_data.reached
	
def base_goto_straight_to_direction(init_x, init_z, x_t, z_t):
	#print("init x, init y, x_t , z_t: ", init_x, init_z, x_t, z_t)
	global init_lock, node_num, init_node
	x = x_t - init_x
	z = z_t - init_z
	if x == 0:
		if z >= 0:
			angle = -3.141/2
		else:
			angle = 3.141/2
	else:
		angle = math.atan(z/x)
		#print("atan: ", angle)
		if x >= 0 and z >= 0:
			angle = angle * -1
		if x < 0 and z >= 0:
			angle = -3.141 - angle
		if x < 0 and z < 0:
			angle = 3.141 - angle
		if x > 0 and z < 0:
			angle = angle * -1

	#file.write("angle: %f " %angle)
	base_goto_set_target(x_t, z_t, angle)
	base_goto_run()
	if base_goto_reached():
		#print("hello")
		base_reset()
		init_lock = 0
		node_num = node_num + 1

def base_run(x,z,alpha):
	global init_lock, init_node
	if init_lock == 0:
		init_node.x = gps.getValues()[0]
		init_node.z = gps.getValues()[2]
	init_lock = 1
	base_goto_straight_to_direction(init_node.x , init_node.z, x, z)
	

#==========================End_code for Driving function============================================================


# create the Robot instance.
commu_channel = 1
# get the time step of the current world.
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
file = open("my_file.txt","w") #open file so the console outputs print statement.
emit = robot.getEmitter("emitter")
receive = robot.getReceiver("receiver")
receive.enable(timestep)
emit.setChannel(commu_channel)

#===========================================================collisionclass============================
#!/usr/bin/python3
import threading

#my_trajectory has a list of nodes.
#each node is a list as [x,y]


my_state = 0 #starting node is always 0
my_trajectory = [] #the whole trajectory for hound, contains nodes, each node is a list of [x,z]
receiv_traj_flag = True #set it true so the robot is waiting to receive its trajecotry

me = "22" #code for receiving hound trajectory from hawk (used in communication),  '11' for hippo
priority_id = "hound" #hounds get to send command to hippo to fall back once deadlock occurs

receive_reply = True #true after hound receives reply (hippo's current node) back from hippo 

my_turn = False #turn once at time 

lock = threading.Lock()
my_id = 2 #hound id is 2, hippo id is 1
my_radius = 0.6 #radius in meter
hippo_radius = 0.6

R2_location = [] #a [x,y] for where the hippo is currently staying

hippo_arrived = False #flag when receives hippo arrived at it goal node
hound_block_state = False #flag when hound is blocked and cant go, so hound passes command to hippo
hippo_block_state = False #flag when hippo is blocked and cant go, so hippo passes command to hound
fall_back = False #is true when hippo has successfully fall back
 
class Collision_avoidance:

	#return current node
	def get_state(self):
		global my_state
		lock.acquire()
		mstate = my_state
		lock.release()
		return mstate
	
	#change the current node
	def set_state(self,index):
		global my_state
		lock.acquire()
		index = int(index)
		print("Hound state changes to ", index)
		my_state = index
		lock.release()
		
	def convert_bin_to_string(self,st):
		str_data = str(st)[2:][:-1]
		return str_data

	def convert_string_to_bin(self,string_):
		return bytes(string_,'utf-8')

	#repply to hippo back when receives quary about location
	def location_handler(self,sender_id):
		st = sender_id + ' ' + 'location_anw'  + ' ' + str(my_trajectory[self.get_state()][0]) + ' ' + str(my_trajectory[self.get_state()][1])
		st = self.convert_string_to_bin(st)
		emit.send(st)
	
	#This function is run in parrallel with the exectute_main function. This is to receives reply from hippo and reply back to hippo 
	def listen(self):
		global my_trajectory, receiv_traj_flag, agreement, my_turn, R2_location, receive_reply, hippo_block_state, fall_back, hippo_arrived
		while robot.step(timestep) != -1:
			#receiving trajecotry from hawk, only once time,
			if receiv_traj_flag:
				while(receive.getQueueLength() > 0):
					bin_data = receive.getData()
					str_data = self.convert_bin_to_string(bin_data)
					str_data = str_data.split()
					#print("string data: ", str_data)
					if str_data[0] == me:
						receiv_traj_flag = False
						#setting my_trajectory
						my_trajectory = self.str_traj_2_reaL_traj(str_data)
						print(my_trajectory)
						#print("hound trajectory length is: ", len(my_trajectory))
						receive.nextPacket()
						break
					receive.nextPacket()	
			else:
				if receive.getQueueLength() > 0:
					bin_data = receive.getData()
					str_data = self.convert_bin_to_string(bin_data)
					str_list = str_data.split()
					#print("hound receiver: ", str_list)
					if float(str_list[0]) == my_id:
						str_list = str_list[1:] #strip my_id field off
						if str_list[0] == "urturn":
							#receive message for me to drive
							if self.get_state() != len(my_trajectory) - 1:
								print("hippo receive my_turn")
							my_turn = True
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "location":
							#receive quary from hippo about my current location
							self.location_handler(str_list[1])
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "location_anw":
							#receive reply back from hippo about it currrent location
							R2_location = [float(str_list[1]), float(str_list[2])]
							receive_reply = True
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "blocked":
							#receive messagee that hippo is blocked and cant go
							hippo_block_state = True
							print("hippo is blocked")
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "unblocked":
							#receive message that hippo is no longer blcoked
							hippo_block_state = False
							receive.nextPacket()
						elif str_list[0] == "arrived":
							#receive messages that hippo has arrived its goal node
							print("hound receives hippo arrived")
							hippo_arrived = True
							receive.nextPacket()	
						elif str_list[0] == "fall":
							#receives message that hippo has successfully fall back
							print("hound receives fall back")
							fall_back = True
							receive.nextPacket()
						else:
							receive.nextPacket()#remove the packet from communication channel	
					else:
						receive.nextPacket()
						

	#A wrapper function for drive function
	#drive function that takes the robot to x,z, Once it reached the state x,z, it sets the reached flag to true
	#This function only return after the car has reached the specified node.
	def goto_target(self,i):
		#drive to the target node in the my_trajectory
		i = int(i)
		finish_one_drive = True
		print("hound is driving to: ", i)
		goto_x, goto_y = my_trajectory[i][0], my_trajectory[i][1]
		while robot.step(timestep) != -1:
			if base_goto_reached() == False or finish_one_drive == True:
				base_run(goto_x, goto_y, 1)
				finish_one_drive = False
			else:
				if i == (len(my_trajectory) - 1):
					self.send_i_arrived()
				print("hound is arriving at state: ", i)
				self.set_state(i) 
				break
				

	#parse the string trajectory to real trajectory
	#each node has the form of [current_x, current_y,]
	def str_traj_2_reaL_traj(self,trajectory):
		real_traj = []
		trajectory = trajectory[1:]
		#print(trajectory)
		node = []
		i = 0
		while (i < len(trajectory)):
			if trajectory[i] == 'n':
				real_traj.append(node)
				node = []
				i = i + 1                   
			elif trajectory[i] == 'e':
				break
			else:
				node.append(float(trajectory[i]))
				i = i + 1
		return real_traj

	#seding message for hippo to go
	def send_your_turn_message(self):
		global my_turn
		my_turn = False
		st = "1 urturn 2"
		st = self.convert_string_to_bin(st)
		emit.send(st)		

	#sending message asking where hippo is, this function waits till listen() receives reply back from hippo. Then it returns the location
	def quary_location(self):
		global R2_location, receive_reply
		st = "1 location 2"
		st = self.convert_string_to_bin(st)
		receive_reply = False
		emit.send(st)
		while robot.step(timestep) != -1:
			#print("hound stuck here")
			if receive_reply == True:
				break
		temp_reply = R2_location
		R2_location = []
		return temp_reply		

	#return true if hippo blocked my way
	def path_has_obstacle(self,R2_info):
		i = self.get_state()
		my_current, my_next = cg.Point(my_trajectory[i][0],my_trajectory[i][1]), cg.Point(my_trajectory[i+1][0],my_trajectory[i+1][1])
		R2_current = cg.Point(R2_info[0],R2_info[1])
		return cg.check_path(my_current, my_next, my_radius, R2_current, hippo_radius)

	#send message to hippo that hippo blocks my way
	def send_iamblocked(self):
		st = "1 blocked 2"
		st = self.convert_string_to_bin(st)
		emit.send(st)			
	
	#send message to hippo that hippo no longer blcoks my way
	def send_iam_unblocked(self):
		st = "1 unblocked 2"
		st = self.convert_string_to_bin(st)
		emit.send(st)
	
	#send message to hippo to fall back during deadlock scenario
	def send_fall_back_command(self):
		st = "1 fall 2"
		st = self.convert_string_to_bin(st)
		emit.send(st)
		
	#send message to hippo that i arrived at my goal node
	def send_i_arrived(self):
		st = "1 arrived 2"
		st = self.convert_string_to_bin(st)
		emit.send(st)	
	
	def execute_main(self):
		global my_turn, hound_block_state, fall_back 
		just_fall_back = False
		#length = len(my_trajectory)
		while robot.step(timestep) != -1:
			if receiv_traj_flag == False:
				#print("hello i'm hound")
				#print(my_trajectory)
				#already received the my_trajectory.
				#i = self.get_state()
				if self.get_state() != (len(my_trajectory) - 1):
					#Not arrived at goal state yet
					#print("hound is stuck here")
					if self.get_state() == 0 and priority_id == "hound":
						self.goto_target(self.get_state()+1)
						self.send_your_turn_message()
					else:
						if hippo_arrived == True:
							self.goto_target(self.get_state()+1)
						elif my_turn == True:
							#if it is my turn to go, check where hippo is 
							R2_info = self.quary_location()
							if self.path_has_obstacle(R2_info):
								hound_block_state = True
								print("hound is blokced")
								if just_fall_back == False:
									#if hippo just fall back, and i'm still blocked then dont give hippo turn.
									self.send_your_turn_message()
								self.send_iamblocked()
							else:
								hound_block_state = False
								self.send_iam_unblocked()
								self.goto_target(self.get_state()+1)
								self.send_your_turn_message()
						else:
							if hound_block_state == True and (hippo_block_state == True or just_fall_back == True):
								print("hound is facing deadlock")
								self.send_fall_back_command()
								just_fall_back = False
								#wait in loop till hippo fall back successfully
								while robot.step(timestep) != -1:
									print("hound is waiting for hippo to finish fall back")
									if fall_back or hippo_block_state == False:
										print ("hound break loop fall back")
										just_fall_back = True
										fall_back = False
										break
				else:
					pass
					#print("Hound has arrived at goal node")
	
	def run(self):
		t1 = threading.Thread(target=self.execute_main)
		t2 = threading.Thread(target=self.listen)
		t1.start()
		t2.start()
		t1.join()
		t2.join()

th = Collision_avoidance()
th.run() #run the two threads in parallel

#======================================================end-collision===================================================	
    
    # Read the sensors:
    
    #emit.send(message2)
    #print("hello emiter \n")    
    #emit.send(message4)
    #print("hello emiter \n") 
    #emit.send(message5)
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
