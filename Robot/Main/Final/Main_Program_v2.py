'''

Main Program v2

--- Features:
 - Support for reading GPS position values through LCM
 - Support for reading GPS command values through LCM

'''

#############
# Import libraries
from multiprocessing import Process, Value, Array
import numpy as np
import matplotlib.pyplot as plt
import time
import serial
import pygame
from random import randint

import lcm
from bumblee import gps_command, action_command

from threading import Timer, Thread

###############################################################
### Define all global variables used here
##############################################################

host_device = '/dev/ttyUSB0'
baud_rate = 57600

# initializes the serial port
myserial = serial.Serial(host_device, baud_rate)
# This variables holds the raw data sent from the arduino
# it is a string data type. Sample data is =
# First 5 values are the Infrared sensors, next 2 the encoders, next 1 is motor status, last two is extras
Data_Unpacked = "0,0,0,0,0,0,0,0,0,0,0"
# stores all the Infrared red values
sensors_io = np.array([0, 0, 0, 0, 0])
# store all the encoder values leftenc = 0, rightenc = 1
encoders = np.array([0, 0])
# store the motor status
motor_status = 0
# stores the remaining IO extra values
extra_io = np.array([0, 0])

# this variable holds the data to be sent out.
Data_toPack = '0,0,0,0,0*'
# variable holds the motor speeds left_speed = 0, right_speed = 1
motor_speeds = np.array([0, 0])
# variable holds the motor enable, 1 = ACTIVATE motor, 0 = Disable motor
motor_enable = 1
# stores the value for the extra IO to be sent out
extra_io_out = np.array([0, 0])

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



# This function uses a timer interrupt or thread to continous send a heartbeat signal
def send_heartbeat():
    while True:
        try:
            # now = time.time()
            # runs every 5 ms or 10 ms
            time.sleep(0.009)
            fetchRecentSerial()
            # print ("heart2: sampling time: ",time.time() - now)
            if global_finish_process.value == 1:
                print(" Heartbeat stopping now")
                break
        except KeyboardInterrupt:
            break

# this need to run countinous with a thread for every 30 - 50 ms interval
def fetchRecentSerial():
    global Data_Unpacked, sensors_io, encoders, motor_status, extra_io, Data_toPack
    # read the values from the serial if available
        
    if myserial.inWaiting():
        try:
            temp = myserial.readline().decode("ascii")

            # check if data is valid or not
            if len(temp) > 5:
                try:
                    Data_Unpacked = temp
                    temp_data = Data_Unpacked.splitlines()
                    temp_data = temp_data[0].split(",")
                    # update the IR sensors values
                    sensors_io = np.array([int(temp_data[0]), int(temp_data[1]), int(temp_data[2]), int(temp_data[3]), int(temp_data[4])])

                    # update the encoders
                    encoders = np.array([np.long(temp_data[5]), np.long(temp_data[6])])

                    # update the motor status
                    motor_status = int(temp_data[7])

                    # update the extra IOs data
                    extra_io = np.array([np.long(temp_data[8]), np.long(temp_data[9])])

                except Exception: # hopefully no error occurs here
                    print(temp)
                    print ("Error with parsing data")
        except Exception:
                pass


    # also we send out the current data available
    # build the data pack together
    #print (motor_speeds, motor_enable, extra_io_out)
    # integrity checker
    rand = randint(0, 100)
    
    Data_toPack = str(rand) + ',' +str(motor_speeds[0]) + ',' + str(motor_speeds[1]) + ',' + str(motor_enable) + ',' + str(extra_io_out[0]) + ',' + str(extra_io_out[1]) + ',' + str(rand) +'*' + '\n'
    # send out the data pack
    myserial.write(Data_toPack.encode())
    

def teleOperateThread():
    while True:
        try:
            # now = time.time()
            # runs every 5 ms or 10 ms
            time.sleep(0.1)
            pygame.event.get()
            teleOperate()
            # print ("heart2: sampling time: ",time.time() - now)
            if global_finish_process.value == 1:
                print(" Teleoperate stopping now")
                break
        except KeyboardInterrupt:
            break

def teleOperate():
    global button_forward, button_turn, button_speedUp, button_speedDown, currentSpeed, motor_speeds
    # get the axis button if value = -1 move forward, if 1 move backward, else stop
    button_forward = (joystick.get_axis(1))
    # if value is -1 turn left if 1 turn right
    button_turn = (joystick.get_axis(0))
    #  value = 1 increase speed up or down by a factor of 1
    button_speedUp = joystick.get_button(9)
    button_speedDown = joystick.get_button(8)
    button_stop = (joystick.get_button(7))
    #print ('dad - ', button_forward, button_turn, button_speedUp, button_speedDown)

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
	global motor_speeds
	motor_speeds = np.array([action.leftspeed, action.rightspeed])
	

##################################################################################
##################################################################################
####################### Main Program Starts Here ################################

# initiaölized LCM
lc = lcm.LCM()

if __name__ == '__main__':

    # initialize the serial communication to be ready
    myserial.reset_input_buffer() # flush out the current state of the serial

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
    extra_io_out[0] = 1

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
    extra_io_out[1] = 1

    count = 0
    
    # running now
    while am_i_ready:
        try:
            # let's do stuff here
            time.sleep(1)         
            if Program_DEBUG:
                print("Recieved - ", Data_Unpacked)
                print ("Sending out - ", Data_toPack)
                print ("GPS Position - ", gps_pose)

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
    myserial.reset_input_buffer()
    myserial.reset_output_buffer
    myserial.close()





