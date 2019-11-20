#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import lcm
from bumblee import gps_command, action_command, collision_robots, platooning

from time import gmtime, strftime, sleep
import math
import numpy as np

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

# function calculates the distance metric
def distanceMetric(robot1, robot2):
    x_diff = robot1[0] - robot2[0]
    y_diff = robot1[1] - robot2[1]

    # calculate distance
    dist = math.sqrt(x_diff ** 2 + y_diff ** 2)

    return dist

# center coordinates for referencing angles from
center_x = 314
center_y = 437
# functions calculates the angle differene with two robots
def angleMetric(robot1):
    # transform pixels to cm
    constX=float(center_x/2.5)
    constY=float(center_y/2.5)
    
    x_diff= float(robot1[0]-constX)
    y_diff=float(robot1[1]-constY)
    # calcuates angle : result is in rad
    angle = np.arctan2(x_diff,y_diff)
    return angle

##################################################
# variable for platooning
myId = spiral_id
# holds the platonnId for each robot
platoonId=0
# role is either follower:2 or leader:4
role=0
startStop=10
# variable for telling which lane the robot should go to
laneId=0
# laneChange command signal. laneChange=0 - stick to current lane, laneChange=1 change to gien laneId
laneChange=0
# the leader speed
mySpeed=0
# Id for the current leader. -1 means no one is set has a leader
currentLeader=-1

# create the LCM message type for platooning
platoon = platooning()
# initialize the default parameters
platoon.myId=spiral_id
platoon.platoonId=0
platoon.role=0
platoon.startStop=0
platoon.laneId=0
platoon.laneChange=0
platoon.mySpeed=0
platoon.currentLeader=0


def platoon_callback(data):
    platoon.startStop = 10
    try:
        p=data.data
        temp=p.split(',')
        print(temp)
        command=temp[0]
        # Platoon ID command
        if(command=='a'):
            for i in range(2,12):
                if(int(temp[i])==platoon.myId):
                    platoonId=int(temp[1])
                    print("Platoon Id:=",platoonId)
                    platoon.platoonId=platoonId
                    break

            #lc.publish("Platooning_Action", platoon.encode())

        # Followers command set
        elif(command=='c'):
            if(platoon.platoonId==int(temp[1])):
                print('in command c')
                for i in range(3,13):
                    if(int(temp[i])==platoon.myId):
                        platoon.role=int(temp[2])
                        print("Following set:=",platoon.role)
                        break

        # Lane ID set        
        elif(command=='d'):
            if(platoon.platoonId==int(temp[1])):
                for i in range(3,13):
                    if(int(temp[i])==platoon.myId):
                        platoon.laneId=int(temp[2])
                        print("Lane Id set:=",platoon.laneId)
                        break
                #lc.publish("Platooning_Action", platoon.encode())
 
        # Set the Leader ID command          
        elif(command=='e'):
            # checks if in the same platoon id
            if(platoon.platoonId==int(temp[1])):
                if(platoon.myId==int(temp[3])):
                    platoon.role=int(temp[2])
                    print("Leader set:=",platoon.role)
                else:
                    platoon.currentLeader=int(temp[3])
                #lc.publish("Platooning_Action", platoon.encode())

        # set the Leader Speed Command
        elif(command=='g'):
            # checks if in the same platoon id
            if(platoon.platoonId==int(temp[1])):
                if(platoon.role==int(temp[2])):
                    platoon.mySpeed=int(temp[3])
                    print("Leader Speed set to:=",platoon.mySpeed)
                #lc.publish("Platooning_Action", platoon.encode())
                    
        # Stops the leader
        elif(command=='h'):
            # checks if in the same platoon id
            if(platoon.platoonId==int(temp[1])):
                if(platoon.role==int(temp[2])):
                    platoon.startStop=int(temp[3])
                    platoon.mySpeed=0
                    print("Leader Stopping")
            #lc.publish("Platooning_Action", platoon.encode())

        ### not being used here. To issue a command to the leader to start or stop
        elif(command=='i'):
            if(platoon.platoonId==int(temp[1])):
                if(platoon.role==int(temp[2])):
                    platoon.startStop=int(temp[3])
                    print("Leader Starting")
            #lc.publish("Platooning_Action", platoon.encode())

        # Lane change command - left
        elif(command=='m'):
            if(platoon.platoonId==int(temp[1])):
                for i in range(3,13):
                    if(int(temp[i])==platoon.myId):
                        platoon.laneChange=int(temp[2])
                print("Lane Change to left")
            #lc.publish("Platooning_Action", platoon.encode())

        # Lance change command - right
        elif(command=='n'):
            if(platoon.platoonId==int(temp[1])):
                for i in range(3,13):
                    if(int(temp[i])==platoon.myId):
                        platoon.laneChange=int(temp[2])
                print("lane Change to right")
            #lc.publish("Platooning_Action", platoon.encode())

        # Change of leader command
        elif (command == 'x'):
            platoon.startStop = int(temp[4])
            if(platoon.platoonId == int(temp[1])):
                if(platoon.myId == int(temp[3])):
                    platoon.role = int (temp[2])

                    print('leader change - new leader:', platoon.role)

                else:
                    platoon.currentLeader=int(temp[3])
        else:
            print('Invalid Command input detected')
                 
        lc.publish("Platooning_Action", platoon.encode())       
    except Exception:
        print ('Error in Platooning')
    
    
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

def gps_callback2(data):
    global gps_x, gps_y, gps_p, gps_q
    # holds the position information for all other robots
    robots_data = []
    robot_x = 0
    robot_y = 0
    robot_id = 0

    distance=0
    angle=0
    
    # this saves all the available information for all robot
    collision_robots = []

    try:
        val=(data.data).split(';')
        for i in range(10):
            temp=val[i].split()
            if(temp[3]==str(spiral_id)):
                # here we check for the integerity
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
        # only send information of all the robots in the arena
        if len(robots_data) > 0:
            for robot in robots_data:
                distance = 0
                # calculate dist between the robots
                distance = distanceMetric([gps.x, gps.y], robot)
                # calculates angle of robots to the center - angle is in degree
                angleOther = np.rad2deg(angleMetric(robot))
                # will save all the available robot distance and angle
                collision_robots.append([robot[2], robot[0], robot[1], distance, angleOther])


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
         # subscriber to the 'Platoon Channel'
        rospy.Subscriber('PlatoonChannel',String, platoon_callback)
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

