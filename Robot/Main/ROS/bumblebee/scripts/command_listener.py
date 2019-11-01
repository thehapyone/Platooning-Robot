#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import lcm
from bumblee import gps_command, action_command

from time import gmtime, strftime, sleep

# Spiral - GPS Configure
# id for the GPS spiral to be use
spiral_id=7
# the GPS variables
gps_x = 0
gps_y = 0
gps_p = 0
gps_q = 0
gps_id = 0

# create the LCM message type for GPS
gps = gps_command()
gps.x = 0
gps.y = 0
gps.p = 0
gps.q = 0

# create the LCM message type for Action Command
action = action_command()
action.leftspeed = 0
action.rightspeed = 0

def action_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    try:            
        p=data.data
        temp=p.split(',')
        temp=temp[5]
        temp=temp.split(';')
        leftspeed= int(temp[0])
        rightspeed=int(temp[1])        
        # publish the result to LCM
        action.leftspeed = leftspeed
        action.rightspeed = rightspeed
        # publish the current message packet
        lc.publish("BumbleBee_Action", action.encode())
    except Exception:
        pass
        
def gps_callback(data):
    global gps_x, gps_y, gps_p, gps_q
    try:
        val=(data.data).split(';')
        for i in range(10):
            temp=val[i].split()
            if(temp[3]==str(spiral_id)):
                # here we check if the integerity
                temp_x = float((temp[0])) / 2.54
                if temp_x != -1:
                    gps_x = temp_x
                    gps_y = float((temp[1])) / 2.54
                    gps_p = (temp[2])
                    gps_q = (temp[4])
                else:
                    break
                break
                
        #print('GPS Position Values: ')
        #print(gps_x, gps_y, gps_p, gps_q)

        # publish the result to LCM
        gps.x = float(gps_x)
        gps.y = float(gps_y)
        gps.p = float(gps_p)
        gps.q = float(gps_q)
        # publish the current message packet
        #lc.publish("BumbleBee_GPS", msg.encode())
        #global msg2
        # fetch the curret time to be sent in the timestamp message

	# publish the current message packet
	lc.publish("BumbleBee_GPS", gps.encode())
        
    except Exception:
        pass

def listen_2_ros_lcm():
    # This function subcribes to the necessary ROS channels
    try:
        # Create the ROS instance
        rospy.init_node('command_listener_bumblebee', anonymous=False)
        # subscribe to the 'Action Channel'
        rospy.Subscriber("action", String, action_callback)
        # subscribe to the 'GPS channel'
        rospy.Subscriber("josefoutput", String, gps_callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        pass


lc = lcm.LCM()

if __name__ == '__main__':
    print ('ROS - LCM Node Launched')
    # start the LCM instance
    try:
        listen_2_ros_lcm()
    except KeyboardInterrupt:
        exit()

