'''

Main Program v5

--- Features:
 - Support for reading GPS position values through LCM
 - Support for reading GPS command values through LCM
 - Support for Mode change
 - Support for infrared lane following - Transfered to uno
 - Support for using LCM to collect sensor readings
 - Serial interface with the Arduino has been disabled

Uses:
    - Uses the Arduino code - Robot_Arduino_v4
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
from bumblee import gps_command, action_command, arduino_in, arduino_out

from threading import Timer, Thread

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

# this variable holds the data to be sent out.
Data_toPack = '0,0,0,0,0,0'
# variable holds the motor speeds left_speed = 0, right_speed = 1
motor_speeds = np.array([0, 0])
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
odo_pose_flip = initial_state
# GPS based position data
gps_pose = [0, 0, 0]

# create data history
odo_pos_stack = []
gps_pos_stack = []
final_pos_stack = []
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


# pattern 
def runpattern():
    global motor_speeds, currentSpeed, current_servo_state
    motor_speeds = np.array([100,100])
    current_servo_state = True
    time.sleep(2)
    motor_speeds = np.array([100,-100])
    current_servo_state = False
    time.sleep(1)
    motor_speeds = np.array([100,100])
    current_servo_state = True
    time.sleep(2)
    motor_speeds = np.array([-100,100])
    current_servo_state = False
    time.sleep(0.9)
    motor_speeds = np.array([100,100])
    current_servo_state = True
    time.sleep(2)
    motor_speeds = np.array([0,0])
    current_servo_state = False

    
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
        time.sleep(0.1)
    elif rightmost > LINETHRESHOLD:
        motor_speeds = np.array([int(reduceFactor*currentSpeed), currentSpeed])
        time.sleep(0.1)

    elif middle_center > LINETHRESHOLD and middle_left > LINETHRESHOLD and middle_right > LINETHRESHOLD:
        currentSpeed = 0

    elif middle_right > LINETHRESHOLD:
        motor_speeds = np.array([int((reduceFactor-0.2)*currentSpeed), currentSpeed])
        time.sleep(0.3)
    elif middle_left > LINETHRESHOLD:
        motor_speeds = np.array([currentSpeed, int((reduceFactor-0.2)*currentSpeed)])
        time.sleep(0.3)
    else:
        motor_speeds = np.array([currentSpeed, currentSpeed])


# This function uses a timer interrupt or thread to continous send a heartbeat signal
def send_heartbeat():
    while True:
        try:
            # now = time.time()
            # runs every 1 ms
            time.sleep(0.01)
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
    
    if button_servo == 1:
        current_servo_state = not(current_servo_state)
        time.sleep(0.1)
    
    # check mode
    if mode_up == -1:
        # manual mode
        robot_mode = 0
    elif mode_up == 1:
        # GPS controller mode
        robot_mode = 1
    elif mode_left == -1:
        # auto mode
        robot_mode = 2
    elif mode_left == 1:
        # pattern mode
        robot_mode = 3

    if robot_mode == 2:
        # auto mode
        # send that value to the uno board
        extra_io_out[1] = 99
        #laneFollowing()
    elif robot_mode == 3:
        runpattern()
        robot_mode = 0
    else:
        # manual mode
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
        if currentSpeed >= 255:
            currentSpeed = 255
            
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

def gps_manager(channel, data):
	gps = gps_command.decode(data)
	'''
	print("Received message on channel \"%s\"" % channel)
	print(msg.x, msg.y, msg.p, msg.q)
	print("")
	'''
	global gps_pose
	gps_pose = [gps.x, gps.y, 0]
	
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
    global sensors_ir, encoders, motor_status, extra_io, Data_Unpacked

    # update the IR sensors values
    sensors_ir = np.array([data_available.extreme_left, data_available.left, data_available.center, data_available.right, data_available.extreme_right])

    # update the encoders
    encoders = np.array([np.long(data_available.encoder_left), np.long(data_available.encoder_right)])

    # update the motor status
    motor_status = data_available.motorEnable

    # update the extra IOs data
    extra_io = np.array([data_available.extra_io1, data_available.extra_io2])

    # for debugging
    Data_Unpacked = str(sensors_ir) + ',' + str(encoders) + ',' + str(motor_status) + ',' + str(extra_io)



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

    # starts the LCM thread
    lcm_thread = Thread(name='Lcm_Manager', target=lcmThread)
    # starts LCM thread
    lcm_thread.setDaemon(True)
    lcm_thread.start()

    if Program_DEBUG:
        print("LCM Thread Started ")
    
    # delay for about 10 ms and activates the greenlight to signal redness
    time.sleep(0.01)
    # activates green light
    #extra_io_out[0] = 1

    # getting the current motor ticks
    if Program_DEBUG:
        print ("Recevied From SERIAL: ", Data_Unpacked)
        print ()
    # get latest encoder value
    pre_left_ticks = encoders[0]
    pre_right_ticks = encoders[1]

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
                print("Encoders - ", encoders)
                print ("Sending out - ", Data_toPack)
                #print ("GPS Position - ", gps_pose)
                print ("Robot Mode - ", robot_mode)

        except KeyboardInterrupt:
            print ("Stop")
            global_finish_process.value = 1
            global_finish = True
            am_i_ready = False
            break

    # cleaning up stage
    heart.join()
    teleOperation.join()
    lcm_thread.join()
    # chill around for some mins
    time.sleep(1)

    pygame.quit()
    print("Finished")




