#!/usr/bin/env python

'''
This Node is in charge of publishing data to the Arduino node topics
It has to be run first before the arduino node starts running

# Arduino Node Publisher
'''

import rospy
from std_msgs.msg import String
import lcm
from bumblee import arduino_out
from random import randint

import time

class ArduinoManager():
    def __init__(self):
       # Create the ROS instance
       rospy.init_node('arduino_sender_bumblebee', anonymous=True)

       # creates the LCM instance
       self.lc = lcm.LCM()
       # channel to publish our commands to arduino node
       self.pub = rospy.Publisher('BumbleBee_Receiver', String, queue_size=5)
       # subcribe to the LCM channel for sending out messages to the Arduino
       self.lc.subscribe("Arduino_In", self.arduino_sendout)

    def update(self):
        # here programs that needs to run or perform some computation.
        try:
            self.lc.handle()
        except Exception:
            self.shutdown()
            exit()

    def arduino_sendout(self, channel, data):
        # create the LCM message for data to be received in
        dataIn = arduino_out.decode(data)
        # integrity checker
        rand = randint(0, 100)
        # here we need to publish the data received.
        Data_toPack = str(rand) + ',' +str(dataIn.leftspeed) + ',' + str(dataIn.rightspeed) + ',' + str(dataIn.motorEnable) + ',' + str(dataIn.io_device1) + ',' + str(dataIn.io_device2) + ',' + str(dataIn.io_device3) + ',' + str(rand)
        #print (Data_toPack)
        # now we publish the data using ros
        self.pub.publish(Data_toPack)


    def spin(self):
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.update()
            time.sleep(0.01)
        rospy.spin()

    def shutdown(self):
        # here we will do cleanup before finally shutdown
        rospy.loginfo("Node is shutting down")
        

def run():
    arduino = ArduinoManager()
    arduino.spin()

if __name__ == '__main__':
    print ('Arduino Command Center - Sender Node Launched')
    try:
        run()
    except KeyboardInterrupt:
        exit()

