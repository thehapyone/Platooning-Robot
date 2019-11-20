#!/usr/bin/env python
import rospy
from time import sleep
from bumblee import arduino_in
from threading import Thread

from timeit import default_timer as timer

import lcm

import sys

import psutil
import subprocess


finshed = 0
node_time = timer()
status_counter = 0

lcm_started = 0
node_restart = 0

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
        proc.communicate()
    process.kill()

#proc = subprocess.Popen(["rosrun bumblebee arduino_node.py", "_port:=/dev/ttyUSB0"], shell=True)

# kill
#kill(proc_pid)


def lcmThread():
    while True:
        try:
            lc.handle()
            #time.sleep(0.01)
            if finshed == 1:
                print(" LCM Shutting down now")
                break
        except KeyboardInterrupt:
            break

# this function is the LCM handler for managing data coming from the sensors
def heartbeat():
    global status_counter
    
    while True:
        sleep(0.1)
        # here i check the last time i got a message
        if (timer() - node_time) > 1 and lcm_started == 1:
            print ('shuttdown down and restarting')
            node_restart = 1
        if finshed == 1:
            break
        
# this function is the LCM handler for managing data coming from the sensors
def arduino_dataIn(channel, data):
    global node_time, status_counter, lcm_started
    # read the available data
    lcm_started = 1
    data_available = arduino_in.decode(data)
    # check the node connection status
    # keep tracking the device time
    node_time = timer()

# starts lccm
lc = lcm.LCM()


def main():
    # runs the python process
    proc = subprocess.Popen(["rosrun bumblebee arduino_node.py", "_port:=/dev/ttyUSB0"], shell=True)
return proc

def kill_previous(procss):
    # kill
    kill(procss)
    
if __name__=="__main__":
   
    # subscribe to the 'BumbleeBee_Receiver Channel'
    lc.subscribe("Arduino_Out", arduino_dataIn)

    # starts the LCM thread
    lcm_thread = Thread(name='Lcm_Manager', target=lcmThread)
    # starts LCM thread
    lcm_thread.setDaemon(True)
    lcm_thread.start()

        # starts the LCM thread
    heartbeat = Thread(name='heartbeat', target=heartbeat)
    # starts heartbeat thread
    heartbeat.setDaemon(True)
    heartbeat.start()
    
    while finshed == 0:
        try:
            my_proc = main()
            while True:
                if node_restart == 1:
                    node_restart = 0
                    break
            lcm_started = 0
            kill_previous(my_proc)
            node_time = timer()
        except KeyboardInterrupt:
            break
        
    
