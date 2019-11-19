#!/usr/bin/env python
from time import sleep
from bumblee import arduino_in
from threading import Thread

from timeit import default_timer as timer

import lcm

import sys

import psutil
import subprocess

from multiprocessing import Process

finshed = 0
node_time = timer()
status_counter = 0

lcm_started = 0
node_restart = 0

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
        #proc.communicate()
    process.kill()

#proc = subprocess.Popen(["rosrun bumblebee arduino_node.py", "_port:=/dev/ttyUSB0"], shell=True)

# kill
#kill(proc_pid)


def lcmThread():
    global finshed
    while True:
        try:
            lc.handle()
            #time.sleep(0.01)
            if finshed == 1:
                print(" LCM Shutting down now")
                break
        except KeyboardInterrupt:
            finshed = 1  
            break

# this function is the LCM handler for managing data coming from the sensors
def heartbeat():
    global status_counter, node_restart, finshed
    
    while True:
        try:
            sleep(0.01)
            # here i check the last time i got a message
            if (timer() - node_time) > 0.5 and lcm_started == 1:
                print ('shuttdown down and restarting')
                node_restart = 1
            if finshed == 1:
                break
        except KeyboardInterrupt:
            finshed = 1  
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


import os
import signal

def main():
    # runs the python process
    proc = subprocess.Popen("gnome-terminal -e \"./run_node.sh\"", stdout=subprocess.PIPE, stderr=None, shell=True)

def kill_previous():
    for process in psutil.process_iter():
        if process.cmdline() == ['python', '/home/ayo/catkin_ws/src/bumblebee/scripts/arduino_node.py']:
            process.terminate()
            break
    
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

    # starts the main prcoess timer
    #node = Process(target=main)
    #node.daemon = True
    #heart.start()

    print ('Started')

    while finshed == 0:
        try:
            main()
            while True:
                if node_restart == 1:
                    node_restart = 0
                    break
            lcm_started = 0
            kill_previous()
            node_time = timer()
            print('now restarted')
        except KeyboardInterrupt:
            break
        
    finshed = 1  
    lcm_thread.join()
    heartbeat.join()
