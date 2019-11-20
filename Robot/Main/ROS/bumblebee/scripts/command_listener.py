#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import lcm
from bumblee import gps_command, action_command, collision_robots

from time import gmtime, strftime, sleep
import math
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

# define the collision radius - 30cm
collison_radius = 30

robots_collis = collision_robots()
robots_collis.robots_len = 4
robots_collis.collision_array = [5, 2, 3, 12]

def distanceMetric(robot1, robot2):
    x_diff = robot1[0] - robot2[0]
    y_diff = robot1[1] - robot2[1]

    # calculate distance
    dist = math.sqrt(x_diff ** 2 + y_diff ** 2)

    return dist

    
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
            print(i)
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

def gps_callback2(data):
    global gps_x, gps_y, gps_p, gps_q
    # holds the position information for all other robots
    robots_data = []
    robot_x = 0
    robot_y = 0
    robot_id = 0

    collision_robots = []
    # this saves all the available distance information for all robot
    robots_distance = []
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
            else:
                #print('dsadsa',i)
                robot_id = int(temp[3])
                # only fetch the valid ones
                if robot_id != -1:
                    robot_x = float((temp[0])) / 2.54
                    robot_y = float((temp[1])) / 2.54
                    robots_data.append([robot_x, robot_y, robot_id])
        
        # at this stage we have the positions of all robots and our own robot
        # only send information of the robots within a 30cm radius to us.
        if len(robots_data) > 0:
            for robot in robots_data:
                distance = 0
                # calculate dist between the robots
                distance = distanceMetric([gps.x, gps.y], robot)
                # will save all the available robot distance
                robots_distance.append([robot[2], robot[0], robot[1], distance]) 
                # check if distance is in the collision radius
                if distance <= collison_radius:
                    # send out
                    collision_robots.append(robot)

        #print('GPS Position Values: ')
        #print(gps_x, gps_y, gps_p, gps_q)

        # publish the result to LCM
        gps.x = float(gps_x)
        gps.y = float(gps_y)
        gps.p = float(gps_p)
        gps.q = float(gps_q)

        #print ('Collision Robots: ', collision_robots)
        # send out the collision robot details
        robots_collis.robots_len = len(collision_robots)
        robots_collis.collision_array = collision_robots
        #mgss = collision_robots()
        #mgss.robots_len = len(collision_robots)
        #mgss.collision_array = collision_robots
        lc.publish("BumbleBee_Collision", robots_collis.encode())
        # publish the current message packet
        lc.publish("BumbleBee_GPS", gps.encode())
    except Exception:
        pass
        

    
def listen_2_ros_lcm():
    # This function subcribes to the necessary ROS channels
    try:
        # Create the ROS instance
        rospy.init_node('command_listener_bumblebee', anonymous=True)
        # subscribe to the 'Action Channel'
        rospy.Subscriber("action", String, action_callback)
        # subscribe to the 'GPS channel'
        rospy.Subscriber("josefoutput", String, gps_callback2)
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

