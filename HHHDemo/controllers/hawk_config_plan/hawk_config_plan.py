"""hawk_controller controller."""

#  You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import ctypes
from matplotlib import pyplot as plt
import numpy as np
from controller import Robot,Camera,CameraRecognitionObject,Compass,GPS,Gyro,InertialUnit,LED,Motor
import planner
from scipy.spatial import distance

# Given four points this function will calculate hoverzone for Hawk
def get_goal_state(corners, fov):
    length = distance.euclidean(corners[0], corners[1])
    width = distance.euclidean(corners[0], corners[3])
    m = max(width, length)
    altitude = .9*m / (2*np.arctan(1/2*fov))
    altitude = round(altitude, 2)
    x = .5*(corners[2][0] + corners[0][0])
    z = .5*(corners[2][1] + corners[0][1])
    run = corners[1][0] - corners[0][0]
    rise = corners[1][1] - corners[0][1]
    if run == 0:
        yaw = np.pi/2
    else:
        yaw = np.tan(rise/run)
    return [x, altitude, z, yaw]

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
camera = robot.getCamera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)
front_left_led = robot.getLED("front left led")
front_right_led = robot.getLED("front right led")
imu = robot.getInertialUnit("inertial unit")
imu.enable(timestep)
gps = robot.getGPS("gps")
gps.enable(timestep)
compass = robot.getCompass("compass")
compass.enable(timestep)
gyro = robot.getGyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getMotor("camera roll");
camera_pitch_motor = robot.getMotor("camera pitch");
front_left_motor = robot.getMotor("front left propeller");
front_right_motor = robot.getMotor("front right propeller");
rear_left_motor = robot.getMotor("rear left propeller");
rear_right_motor = robot.getMotor("rear right propeller");
motors = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};

# Get Hover Zone
corners = [(-2.75, -1.75), (2.75, -1.75), (2.75,1.75), (-2.75, 1.75)]
hover_zone = get_goal_state(corners, 1)
target_altitude = hover_zone[1]
target_x = hover_zone[0]
target_z = hover_zone[2]

# Initialize motors
for m in motors:
    m.setPosition(math.inf);
    m.setVelocity(1.0);

# Everyone Waits a second
while (robot.step(timestep) != -1):
    if (robot.getTime() > 1.0):
        break

print("Start the drone...");

# Constants for the PID controller
roll_trim = 0.07 #positive is right
pitch_trim = 0.15 #positive is backwards
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0
killswitch = 0.0

target_yaw = imu.getRollPitchYaw()[2] +.02


camx=camera.getWidth()
camy=camera.getHeight()
configSpace = []
for k in range(camx):
    configtemp = []
    for l in range(camy):
        configtemp.append(0)
    configSpace.append(configtemp)

# Intitialize state machine varibles and other constants
request = True
hover1 = 0
y_good = False
x1 = 0
x_good = False
z1 = 0
z_good = False

# Main loop:
# - perform simulation steps until Webots is stops the controller
while robot.step(timestep) != -1 and killswitch != 1:
    
    roll = imu.getRollPitchYaw()[0] + (math.pi / 2.0)
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    altitude = gps.getValues()[1]
    x = gps.getValues()[0]
    z = gps.getValues()[2]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]

    # Pretty Lights
    time = robot.getTime()
    led_state = int(time) % 2
    front_left_led.set(led_state)
    front_right_led.set(~led_state)

    # Controls Camera Shakiness
    camera_roll_motor.setPosition( -0.115 * roll_acceleration)
    camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)

    # Setup for inputs
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    
    # Calculates Errors in Position
    altitude_error = abs(altitude - target_altitude)
    x_error = target_x - x
    z_error = target_z - z

    # Altitide correction: Threshold of Error and Time for verifications can be changed
    if altitude_error < .4 and not y_good:
        hover1 = hover1 + 1
        if hover1 > 100:
            y_good = True
            print('Achieved altitude')
    elif altitude_error >= .5:
        hover1 = 0
        y_good = False

    # x correction: Threshold of Error and Time for verifications can be changed
    if y_good:
        if abs(x_error) < .1 and not x_good:
            x1 = x1 + 1
            if x1 > 150:
                x_good = True
                print('Achieved x')
        elif abs(x_error) >= .1:
            x1 = 0
            x_good = False
            pitch_disturbance = max(min(x_error, 2), -2)
    
    # z correction: Threshold of Error and Time for verifications can be changed
    if y_good and x_good:
        if abs(z_error) < .04 and not z_good:
            z1 = z1 + 1
            if z1 > 150:
                print('Achieved z')
                z_good = True
                #request = True
        elif abs(z_error) >= .04:
            z1 = 0
            z_good = False
            roll_disturbance = -max(min(z_error, 1.5), -1.5)      
           
    # Valid Hover_mode Requires x y and z good 
    if y_good and x_good and z_good:
        hover_mode = True
    else:
        hover_mode = False

    # Only Satisfy requests if there is one, you're in hover mode, and not done(although there
    # should be no requests if u are done)
    if request and hover_mode:
        # This next batch of code handles setting up the input to the RRT and configuration space
        number_of_objects = camera.getRecognitionNumberOfObjects();
        if number_of_objects > 0:
                objects = camera.getRecognitionObjects()
        for k in range(camx):
            for l in range(camy):
                configSpace[k][l] = 0
       #using webots super camera
        for i in objects:
            pptr = int(i.position)
            position = ctypes.c_double * 3
            position = position.from_address(pptr)
            optr = int(i.orientation)
            orientation = ctypes.c_double * 4
            orientation = orientation.from_address(optr)
            piptr = int(i.position_on_image)
            position_on_image = ctypes.c_int * 2
            position_on_image = position_on_image.from_address(piptr)
            siptr = int(i.size_on_image)
            size_on_image = ctypes.c_int * 2
            size_on_image = size_on_image.from_address(siptr)
            centx =position_on_image[0]
            centy = position_on_image[1]
            xdist = math.ceil(size_on_image[0]/2)
            ydist = math.ceil(size_on_image[1]/2)
            if(i.get_colors() == [1,1,1]):
                houndstart = [centx,camy - centy]
            elif (i.get_colors() == [0, 0, 0]):
                hippostart = [centx,camy - centy]
            elif (i.get_colors() == [0, 1, 0]):
                gggoal = [centx,camy - centy]
            elif(i.get_colors() == [1,0,0]):
                #getting configspace for obstacles
                for cdx in range(centx-xdist,centx+xdist):
                    for cdy in range(centy-ydist,centy+ydist):
                        if(cdx>=0 and cdx<camx and cdy>=0 and cdy<camy):
                            configSpace[cdx][cdy] = 1;
        data = np.array(configSpace)
        data = np.transpose(data)
        
        # Calling to map the RRT (you get five chances each)
        chances = 0
        while chances < 2:
            pathHD = planner.runRRT('HOUND', [24,48], data, houndstart + [0],gggoal + [0])
            if pathHD != []:
                break
            chances = chances + 1
        
        chances = 0
        while chances < 2:
            pathHP = planner.runRRT('HIPPO', [96,136], data, hippostart+ [0],gggoal + [0])
            if pathHP != []:
                break
            chances = chances + 1
        # Reset request variables
        request = False

    # Variables that effect propeller inputs
    clamped_difference_yaw = max(min(target_yaw - yaw, 1), -1)
    yaw_input = -clamped_difference_yaw
    roll_input = k_roll_p * max(min(roll, 1), -1) + roll_acceleration + roll_disturbance + roll_trim
    pitch_input = k_pitch_p * max(min(pitch, 1), -1) - pitch_acceleration + pitch_disturbance - pitch_trim
    clamped_difference_altitude = max(min(target_altitude - altitude + k_vertical_offset, 1), -1)
    vertical_input = k_vertical_p * math.pow(clamped_difference_altitude, 3.0)

    # Setup up propeller inputs
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input

    # Inputs
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)
