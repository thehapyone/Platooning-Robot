'''

Main Program v9

--- Features:
 - Support for reading GPS position values through LCM
 - Support for reading GPS command values through LCM
 - Support for Mode change
 - Support for infrared lane following - Transfered to uno
 - Support for Odometry. 
 - Serial interface with the Arduino has been disabled
 - Fuse together odometry and GPS values to give us better
 estimate of the robot location and orientation
 - Fixed speed update bug on lane following
 - Add support for reading ultrasonic distance
 - PID controller for keeping distance to ultrasonic implemented
 - Support for lane change implemented: changing to the left and changing to the right.
 ------------------------------------------------------------------------------------------
 - Support for Cyborg mode controlled. : Robot receives information about the cyborg information
 through LCM at attempts to move towards it while keeping a set distance to the object.
 
Uses:
    - Uses the Arduino code - Robot_Arduino_v5
'''

#############
# Import libraries
from multiprocessing import Process, Value, Array
import numpy as np
import matplotlib.pyplot as plt
import time
import pygame
from random import randint

import lcm
from bumblee import gps_command, action_command, arduino_in, arduino_out, collision_robots, cyborg_detection

from threading import Timer, Thread
from Odometry import get_motor_state

from Robot_Helper import ThreadWithReturnValue

###############################################################
### Define all global variables used here
##############################################################

# This variables holds the raw data sent from the arduino
# it is a string data type. Sample data is =
# First 5 values are the Infrared sensors, next 2 the encoders, next 1 is motor status, last two is extras
Data_Unpacked = "0,0,0,0,0,0,0,0,0,0,0"
# stores all the Infrared red values
sensors_ir = np.array([0, 0, 0, 0, 0])
# store all the encoder values leftenc = 0, rightenc = 1
encoders = np.array([0, 0])
# store the motor status
motor_status = 0
# stores the remaining IO extra values
extra_io = np.array([0, 0])
# store the ultrasonic sensor distance being sent
sensor_distance = 0

# this variable holds the data to be sent out.
Data_toPack = '0,0,0,0,0,0'
# variable holds the motor speeds left_speed = 0, right_speed = 1
motor_speeds = np.array([0, 0])
controller_speeds = [0, 0]
# variable holds the motor enable, 1 = ACTIVATE motor, 0 = Disable motor
motor_enable = 1
# stores the value for the extra IO to be sent out
extra_io_out = np.array([0, 0, 0])

#############
#############
# Button variables
button_forward = 0
button_turn = 0
button_speedUp = 0
button_speedDown = 0
currentSpeed = 0

pygame.init()


############## Odometery and GPS position related stuffs
# ticks variables
pre_left_ticks = 0
pre_right_ticks = 0

new_left_ticks = 0
new_right_ticks = 0

# Measured start position. (x, y, theta)
initial_state = [0, 0, 0]
# Covariance at start position. intial convaraince of 10,10,5 deg
odo_cov = np.diag([10 ** 2, 10 ** 2, np.deg2rad(1) ** 2])
# odometry based position data
odo_pose = initial_state

# tells us if their is information from the GPS
gps_available = False
# GPS based position data
gps_pose = [0, 0, 0]

# create data history
odo_pose_stack = []
gps_pose_stack = []
final_pose_stack = []
odo_cov_stack = []
gps_cov_stack = []
final_cov_stack = []

data_logger_odo = []
data_logger_gps = []
data_logger_eky = []

###################################################
# GLobal readness
am_i_ready = False
# handles terminating other threads and process
global_finish = False
# used across multiple running process
global_finish_process = Value('i', 0)
global_finish_process.value = 0

weAreDoneHere = False
# for debugging
Program_DEBUG = True

### GPS Server related variables

LINETHRESHOLD = 800
reduceFactor = 0.35

# robot mode
# mode 0 - Manual control
# mode 1 - GPS control
# mode 2 - Line Following
robot_mode = 0

# servo state
current_servo_state = False

#robot collision data
robots_collision = list()

# distanceThread
distanceThread = False
# min working speed
speed_min_robot = 35
speed_max_robot = 100

set_point = 15

# variable for storing the insect detections
insect_detections = list()
# for handling the cyborg thread
stop_cyborg = True
cyborg_speed = [0,0]

# function keeps distance to the nearest robot
# this should run as a thread when it is being called
def keepDistance():

    global controller_speeds

    # the PID parameters
    kp = 5
    ki = 0
    kd = 1.5

    integral = 0
    derivative = 0
    lasterror = 0

    time_change = 0.1

    while distanceThread:
        
        # find the error difference
        error = sensor_distance - set_point
        
        integral = (integral + error)
        derivative = (error - lasterror)/time_change
        lasterror = error

        pid_speed = kp * error + integral * ki + derivative * kd
        print ('pid speed - :', pid_speed)
        print ('error - :', error)
        print ('distance - :', sensor_distance)

        # now we need to interpolate this speed value to something that won't kill the robot

        if pid_speed > 0:
            sign = 1
        else:
            sign = -1

        # need to find the max speed value and the min speed value pid results

        # check if currentSpeed is low or not set
        if currentSpeed <= speed_min_robot:
            max_speed = speed_min_robot + 30
        else:
            max_speed = currentSpeed
        
        if sign == 1:
            outputSpeed = int(np.interp(pid_speed, [0, 600], [speed_min_robot, max_speed]))
        else:
            outputSpeed = int(np.interp(pid_speed, [-100, 0], [-abs(max_speed), -speed_min_robot]))

        #print (outputSpeed)
        # send out the speed
        # here, we stop the robot if the error difference is less than 3
        if abs(error) <= 3:
            controller_speeds = [0, 0]
        else:
            controller_speeds = [outputSpeed, outputSpeed]
        print (controller_speeds)
        time.sleep(time_change)
        

# transform odometry postion - function transform odometry value to the real world coordinates
def transformOdo(position, mode):
    x, y, theta = position
    # two types of mode are supported here.
    # transforming to the world or transform back to the orginal coordinates
    # mode 1 transform to the world (GPS) coordinates
    if mode == 1:
        newX = -1 * y
        newY = x
        return (newX, newY, theta)
    # transform from the world (GPS) coordinates to odometry coordinates
    elif mode == 2:
        newX = y
        newY = -1 * x
        return (newX, newY, theta)

    else:
        return (newX, newY, theta)

# Odometry Thread
def OdoThread():
    global pre_left_ticks, pre_right_ticks

    # get latest encoder value
    new_left_ticks = encoders[0]
    new_right_ticks = encoders[1]
    
    motor_ticks = ((new_left_ticks - pre_left_ticks), (new_right_ticks - pre_right_ticks))
    # gets the global saved pisition data
    # first transform position back to odo coordinates
    pose_temp = transformOdo(odo_pose, mode=2)
    # updates the latest value
    pose, cov = get_motor_state(pose_temp, motor_ticks, odo_cov)
    
    # here will update the ticks of the robot
    pre_left_ticks = new_left_ticks
    pre_right_ticks = new_right_ticks
    # odo_pose = pose
    return pose, cov

# Kalmar Filter thread
def KalmarFilter():
    # A queue to hold position related data availables
    # define the global variables that needs to be updated
    global odo_pose, odo_cov, odo_pose_flip, odo_pose_stack, gps_pose_stack

    # EKY Threads always runs until told to stop

    print("Kalmar launched")
    # get the current time for reference purpose
    time_ref = time.time()
    while True:
        try:
            if global_finish_process.value == 1:
                break
            # now = time.time()
            # starts the odo_thread
            odo_thread = ThreadWithReturnValue(name="OdometryThread", target=OdoThread)
    
            # starts the odo thread now
            odo_thread.start()
    
            # waits for it to finish
            pose, odo_cov = odo_thread.join()
            # transform the value to the GPS coordinates system
            odo_pose = transformOdo(pose, mode=1)

            # updates data history
            odo_pose_stack.append(odo_pose)
            gps_pose_stack.append(gps_pose)
            
            # wait for 20 ms
            time.sleep(0.02)

        except KeyboardInterrupt:  # handles error
            print("Exception in Kalmar")
            break

def saveLogger():
    # creates the odo array and save to text
    odo_logs = np.array([(log[0], log[1], log[2], log[3], log[4], log[5], log[6], log[7], log[8], log[9], log[10], log[11], log[12]) for log in data_logger_odo])
    lidar_logs = np.array([(log[0], log[1], log[2], log[3], log[4], log[5], log[6], log[7], log[8], log[9], log[10], log[11], log[12]) for log in data_logger_lidar])
    eky_logs = np.array([(log[0], log[1], log[2], log[3], log[4], log[5], log[6], log[7], log[8], log[9], log[10], log[11], log[12]) for log in data_logger_eky])

    f_odo = open("odologs.txt", "wb")
    f_lidar = open("lidarlogs.txt", "wb")
    f_eky = open("ekylogs.txt", "wb")

    # save results
    np.savetxt(f_odo, odo_logs)
    np.savetxt(f_lidar, lidar_logs)
    np.savetxt(f_eky, eky_logs)

    f_odo.close()
    f_lidar.close()
    f_eky.close()

    
# this function handles the robot lane following.
def laneFollowing():
    '''
    The flow is like this:
     - Use the IR sensors to detect the black track.
     - Check if the leftmost sensor detects a black track, if yes,
     turn the robot towards the right side by some offset
     - check if the rightmost sensor detects a black track, if yes,
     turn the robot towards the left side by some offset
     - check if both the middle three sensors are detecting the black track,
     then stop the robot from moving.
     - if the left most, right most, and middle sensors are not detecting anything then
     - move forward.
    :return:
    '''
    global motor_speeds, currentSpeed
    # asign the sensor variables
    leftmost, middle_left, middle_center, middle_right, rightmost = sensors_ir

    if leftmost > LINETHRESHOLD:
        motor_speeds = np.array([currentSpeed, int(reduceFactor*currentSpeed)])
        time.sleep(0.2)
    elif rightmost > LINETHRESHOLD:
        motor_speeds = np.array([int(reduceFactor*currentSpeed), currentSpeed])
        time.sleep(0.2)

    elif middle_center > LINETHRESHOLD and middle_left > LINETHRESHOLD and middle_right > LINETHRESHOLD:
        currentSpeed = 0
        motor_speeds = np.array([currentSpeed, currentSpeed])

    elif middle_right > LINETHRESHOLD:
        motor_speeds = np.array([0, currentSpeed])
        time.sleep(0.5)
    elif middle_left > LINETHRESHOLD:
        motor_speeds = np.array([currentSpeed, 0])
        time.sleep(0.5)
    else:
        motor_speeds = np.array([currentSpeed, currentSpeed])


def cyborg_mode():
    # this functions handles things related to controlling the robot to the insects
    # the PID parameters

    global stop_cyborg, cyborg_speed
    kp = 3
    ki = 0
    kd = 1.5

    integral = 0
    derivative = 0
    lasterror = 0

    time_change = 0.1

    # Control parameter
    base_length = 15.7
    R = 3.5

    # fetch the latest detection results
    # only the first value should be used
    # check if their is data available first
    if len(insect_detections) > 0:
        _, insect_angle, insect_distance = insect_detections[0]

        print ('now in insect mode: ',insect_detections[0])
        # only follow to about 20 cm distance
        while insect_distance > 10 and insect_angle != -1:
            try:            
                # fetch the latest value
                if len(insect_detections) == 0:
                    stop_cyborg = True
                    break
                
                _, insect_angle, insect_distance = insect_detections[0]
                
                theta = odo_pose[2]
                # find the angle that my robot needs to turn to
                alpha = insect_angle

                # we have two goals, optimized the angle to almost zero. i.e the robot is perfectly algin to the detected object
                # and reduce the distance to object.

                # the angular velocity to get to goal
                v = 50

                integral = (integral + alpha)
                derivative = (alpha - lasterror)/time_change
                lasterror = alpha

                # here we also check if the error is slow, just go straghit
                if abs(alpha) <= 10:
                    rightSpeed = v
                    leftSpeed = v

                else:                    
                    pid_angle = kp * alpha + integral * ki + derivative * kd
                    # calc left and right velocity
                    Vr = (2 * v + (pid_angle * base_length)) / 2 * R / 2
                    Vl = (2 * v - (pid_angle * base_length)) / 2 * R / 2

                    rightSpeed = np.interp(Vr, [-3000, 3000], [-100, 100])
                    leftSpeed = np.interp(Vl, [-3000, 3000], [-100, 100])

                # check if currentSpeed is low or not set
                if currentSpeed <= speed_min_robot:
                    max_speed = speed_min_robot + 30
                else:
                    max_speed = currentSpeed

                
                if rightSpeed > 0:
                    outputSpeed_right = int(np.interp(rightSpeed, [0, 100], [speed_min_robot, max_speed]))
                else:
                    outputSpeed_right = int(np.interp(rightSpeed, [-100, 0], [-abs(max_speed), -speed_min_robot]))

                if leftSpeed > 0:
                    outputSpeed_left = int(np.interp(leftSpeed, [0, 100], [speed_min_robot, max_speed]))
                else:
                    outputSpeed_left = int(np.interp(leftSpeed, [-100, 0], [-abs(max_speed), -speed_min_robot]))

                
                # debugging
                print ('error :',alpha)
                print ('distance to object:',insect_distance)
                #print ('Vr and vl: ', (Vr,Vl))
                print ('Left and Right speed values', (leftSpeed, rightSpeed))
                print ('Left and Right speed values (updated)', (outputSpeed_left, outputSpeed_right))

                # cyborg speed update
                cyborg_speed = [outputSpeed_left, outputSpeed_right]

                time.sleep(time_change)

                if stop_cyborg == True:
                    break
            except Exception:
                print ('Error in Cyborg mode')
                
        # reset cyborg mode to being ready again
        stop_cyborg = True
        # should consider stoping the robot if this loop ends
        
# This function uses a timer interrupt or thread to continous send a heartbeat signal
def send_heartbeat():
    while True:
        try:
            # now = time.time()
            # runs every 5 ms or 10 ms
            time.sleep(0.009)
            sendUpdate()
            # print ("heart2: sampling time: ",time.time() - now)
            if global_finish_process.value == 1:
                print(" Heartbeat stopping now")
                break
        except KeyboardInterrupt:
            break

# this need to run countinous with a thread for every 30 - 50 ms interval
def sendUpdate():
    global extra_io_out, Data_toPack

    # also we send out the current data available
    # build the data pack together
    #print (motor_speeds, motor_enable, extra_io_out)

    # updates servo value
    if current_servo_state == True:
        extra_io_out[0] = 150
    else:
        extra_io_out[0] = 300

    dataOut = arduino_out()
    
    dataOut.leftspeed = motor_speeds[0]
    dataOut.rightspeed = motor_speeds[1]
    dataOut.motorEnable = motor_enable

    dataOut.io_device1 = extra_io_out[0]
    dataOut.io_device2 = extra_io_out[1]
    dataOut.io_device3 = extra_io_out[2]
    
    # send out the data pack
    lc.publish("Arduino_In", dataOut.encode())

    # for debugging
    Data_toPack = str(dataOut.leftspeed) + ',' + str(dataOut.rightspeed) + ',' + str(dataOut.motorEnable) + ',' + str(dataOut.io_device1) + ',' + str(dataOut.io_device2) + ',' + str(dataOut.io_device3) 

    # reset the lane change values being sent so it doens't keep issuing that commands
    '''
    if extra_io_out[2] == 1 or extra_io_out[2] == 2:
        # then reset those values
        extra_io_out[2] = 0
    '''

def teleOperateThread():
    while True:
        try:
            # now = time.time()
            # runs every 5 ms or 10 ms
            time.sleep(0.02)
            pygame.event.get()
            teleOperate()
            # print ("heart2: sampling time: ",time.time() - now)
            if global_finish_process.value == 1:
                print(" Teleoperate stopping now")
                break
        except KeyboardInterrupt:
            break

def teleOperate():
    global button_forward, button_turn, button_speedUp, button_speedDown, currentSpeed, motor_speeds, robot_mode, current_servo_state

    global distanceThread, extra_io_out, stop_cyborg
    
    # get the axis button if value = -1 move forward, if 1 move backward, else stop
    button_forward = (joystick.get_axis(1))
    # if value is -1 turn left if 1 turn right
    button_turn = (joystick.get_axis(0))
    #  value = 1 increase speed up or down by a factor of 1
    button_speedUp = joystick.get_button(9)
    button_speedDown = joystick.get_button(8)
    button_stop = (joystick.get_button(7))
    #print ('dad - ', button_forward, button_turn, button_speedUp, button_speedDown)

    # gets hat value
    hats = joystick.get_hat(0)
    mode_up = hats[1]
    mode_left = hats[0]

    # gets select and start button
    button_select = joystick.get_button(10)
    button_start = joystick.get_button(11)

    # servo button
    button_servo = joystick.get_button(0)

    # lane change button
    button_lane_left = joystick.get_button(13)
    button_lane_right = joystick.get_button(14)

    
    if button_servo == 1:
        current_servo_state = not(current_servo_state)
        time.sleep(0.1)
    
    # check mode
    if mode_up == -1:
        # manual mode
        robot_mode = 0
    elif mode_up == 1:
        # GPS controller mode ---> now cyborg mode
        robot_mode = 1
    elif mode_left == -1:
        # auto mode
        robot_mode = 2
    elif mode_left == 1:
        # pattern mode ---> now keeping distance mode
        robot_mode = 3

    if robot_mode == 2:
        # auto mode
        # send that value to the uno board
        stop_cyborg = True
        
        extra_io_out[1] = 99
        # only update the motor speed manually if keep distance isn't running
        if distanceThread == False:
            motor_speeds = np.array([currentSpeed, currentSpeed])
        # check for lane change buttons being pressed
        if button_lane_left == 1:
            extra_io_out[2] = 1
        if button_lane_right == 1:
            extra_io_out[2] = 2
            
    elif robot_mode == 3:
        #runpattern()
        # run the keep distance as a thread and let it do it's things
        if distanceThread == False:
            distanceThread = True
            distanceController = Thread(target=keepDistance)
            distanceController.start()
        #robot_mode = 2
        # update the motor speed value from the controller thread
        if distanceThread == True:
            motor_speeds = np.array([controller_speeds[0], controller_speeds[1]])
            
    elif robot_mode == 1:
        # cyborg mode
        if stop_cyborg == True:
            stop_cyborg = False
            cyborgController = Thread(target=cyborg_mode)
            cyborgController.start()

        if stop_cyborg == False:
            # update the motor speed
            motor_speeds = np.array([cyborg_speed[0], cyborg_speed[1]])
    else:
        # manual mode
        distanceThread = False
        stop_cyborg = True
        
        extra_io_out[1] = 0
        if button_forward < -0.5:
            # time to move forward:
            speed = currentSpeed
            motor_speeds = np.array([speed, speed])
        elif button_forward > 0.5:
            speed = -currentSpeed
            motor_speeds = np.array([speed, speed])
            
        if button_stop == 1:
            currentSpeed = 0
            motor_speeds = np.array([currentSpeed, currentSpeed])

        if button_turn < -0.5:
            # time to turn left:
            speed = currentSpeed
            motor_speeds = np.array([-speed, speed])
        elif button_turn > 0.5:
            # turn right
            speed = currentSpeed
            motor_speeds = np.array([speed, -speed])
        

    if button_forward != 0 and button_speedUp == 1:
        # increase the speed
        currentSpeed = currentSpeed + 1
        if currentSpeed >= speed_max_robot:
            currentSpeed = speed_max_robot
            
    # decreases the speed
    if button_forward != 0 and button_speedDown == 1:
        currentSpeed = currentSpeed - 1
        if currentSpeed <= 0:
            currentSpeed = 0


def lcmThread():
    while True:
        try:
            lc.handle()
            #time.sleep(0.01)
            if global_finish_process.value == 1:
                print(" LCM Shutting down now")
                break
        except KeyboardInterrupt:
            break

def insect_manager(channel, data):
    insects = cyborg_detection.decode(data)
    
    global insect_detections
    insect_detections = insects.data
    #print (insect_detections)

def collison_manager(channel, data):
    mgs = collision_robots.decode(data)
    '''
    print("Received message on channel \"%s\"" % channel)
    print(msg.x, msg.y, msg.p, msg.q)
    print("")
    '''
    robot_len = mgs.robots_len
    global robots_collision
    robots_collision = mgs.collision_array

    # transform the robots collision data to be able to find the location of each robots
    '''
    data collision structure
    robot id, distance, location
    0         10cm       FRONT
    3         20cm       BACK
    8         28cm       LEFT
    '''

    #if robot_len > 0:

def gps_manager(channel, data):
    gps = gps_command.decode(data)
    '''
    print("Received message on channel \"%s\"" % channel)
    print(msg.x, msg.y, msg.p, msg.q)
    print("")
    '''
    global gps_pose, gps_available
    gps_pose = (gps.x, gps.y, 0)
    gps_available = True

    global odo_pose
    ### Here we will fuse together both the GPS and Odo position
    ### since we believe the GPS value is super accurate,
    ### we automatically take the values of the GPS when available
    odo_pose = (gps.x, gps.y, odo_pose[2])
    
def action_manager(channel, data):
    action = action_command.decode(data)
    '''
    print("Received message on channel \"%s\"" % channel)
    print(action.leftspeed, action.rightspeed)
    print("")
    '''
    global motor_speeds, currentSpeed
    # only update if in GPS mode
    if robot_mode == 1:
            if action.leftspeed == action.rightspeed:
                currentSpeed == action.rightspeed
                motor_speeds = np.array([action.leftspeed+10, action.rightspeed+10])
            elif action.leftspeed > action.rightspeed:
                motor_speeds = np.array([action.leftspeed+10, action.rightspeed])
            else:
                motor_speeds = np.array([action.leftspeed, action.rightspeed+10])

# this function is the LCM handler for managing data coming from the sensors
def arduino_dataIn(channel, data):
    # read the available data
    data_available = arduino_in.decode(data)
    global sensors_ir, encoders, motor_status, extra_io, Data_Unpacked, sensor_distance

    # update the IR sensors values
    sensors_ir = np.array([data_available.extreme_left, data_available.left, data_available.center, data_available.right, data_available.extreme_right])

    # update the encoders
    encoders = np.array([np.long(data_available.encoder_left), np.long(data_available.encoder_right)])

    # update the motor status
    motor_status = data_available.motorEnable

    # update the extra IOs data
    extra_io = np.array([data_available.extra_io1, data_available.extra_io2])

    # update the sensor distance read
    temp = data_available.distance
    if temp < 450:
        sensor_distance = temp
    # for debugging
    Data_Unpacked = str(sensors_ir) + ',' + str(encoders) + ',' + str(motor_status) + ',' + str(extra_io) + ',' + str(sensor_distance)


##################################################################################
##################################################################################
####################### Main Program Starts Here ################################

# initiaölized LCM
lc = lcm.LCM()

if __name__ == '__main__':

    # starts the send_heartbeat timer thread
    heart = Thread(name='HeartBeat', target=send_heartbeat)
    # starts heatbeat siganl
    heart.setDaemon(True)
    heart.start()

    # initialize our joystick
    # joystick config
    # Initialize the joysticks.
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
  
    # starts the send_teleoperate thread
    teleOperation = Thread(name='TeleOperate', target=teleOperateThread)
    # starts TeleOperate siganl
    teleOperation.setDaemon(True)
    teleOperation.start()

    # Starts the LCM Channels for handling GPS stuffs
    lc = lcm.LCM()

    lc.subscribe("BumbleBee_Action", action_manager)
    lc.subscribe("BumbleBee_GPS", gps_manager)
    lc.subscribe("Arduino_Out", arduino_dataIn)
    lc.subscribe("BumbleBee_Collision", collison_manager)
    lc.subscribe("BumbleBee_Insects", insect_manager)

    # starts the LCM thread
    lcm_thread = Thread(name='Lcm_Manager', target=lcmThread)
    # starts LCM thread
    lcm_thread.setDaemon(True)
    lcm_thread.start()

    if Program_DEBUG:
        print("LCM Thread Started ")

    # needs to wait to ensure LCM has connected to the data streaming source
    time.sleep(2)
    # get latest encoder value
    pre_left_ticks = encoders[0]
    pre_right_ticks = encoders[1]

    # here we will update the inital value for the robot from the GPS position given
    # update initial odo_pose
    # waits for 5 secs for GPS to be ready
    time.sleep(5)
    if gps_available == True:
        odo_pose = (gps_pose[0], gps_pose[1], odo_pose[2])

    # starts the kalmar thread
    kalmar = Thread(name="KalmarThread", target=KalmarFilter)
    kalmar.setDaemon(True)
    kalmar.start()
    
    # delay for about 10 ms and activates the greenlight to signal redness
    time.sleep(0.1)
    # activates green light
    #extra_io_out[0] = 1

    # Now I am ready
    am_i_ready = True
    if Program_DEBUG:
        print("Now am Ready", am_i_ready)
        print ()

    # activates the running light
    #extra_io_out[1] = 1
    
    # running now
    while am_i_ready:
        try:
            # let's do stuff here
            time.sleep(1)         
            if Program_DEBUG:
                print("Recieved - ", Data_Unpacked)
                #print ("Odometry Position - ", odo_pose)
                #print ("Sending out - ", Data_toPack)
                #print ("GPS Position - ", gps_pose)
                print ("Robot Mode - ", robot_mode)

        except KeyboardInterrupt:
            print ("Stop")
            global_finish_process.value = 1
            global_finish = True
            am_i_ready = False
            break

    # cleaning up stage
    distanceThread = False
    heart.join()
    teleOperation.join()
    lcm_thread.join()
    kalmar.join()
    # chill around for some mins
    time.sleep(1)

    pygame.quit()

    print("odo pose - ", odo_pose)
    print("GPS pose - ", gps_pose)
    print("Finished")

        ####################### Plotting resutls######################
    pos_x = []
    pos_y = []
    pos_x_2 = []
    pos_y_2 = []
    pos_x_3 = []
    pos_y_3 = []
    for i in odo_pose_stack:
        pos_x.append(i[0])
        pos_y.append(i[1])
    for i in gps_pose_stack:
        pos_x_2.append(i[0])
        pos_y_2.append(i[1])
    #for i in final_pose_stack:
    #    pos_x_3.append(i[0])
    #    pos_y_3.append(i[1])

    '''

    plt.figure()
    plt.plot(pos_x, pos_y, "r-", label='Odometry Movement')
    plt.plot(pos_x_2, pos_y_2, "g-", label='GPS Movement')
    #plt.plot(pos_x_3, pos_y_3, "b-")

    #plt.plot(refscan[:, 0], refscan[:, 1], 'b.')

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()
    '''




