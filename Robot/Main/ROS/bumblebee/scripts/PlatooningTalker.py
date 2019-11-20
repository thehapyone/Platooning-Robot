#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from curtsies import Input
def talker():
    pub=rospy.Publisher('PlatoonChannel',String,queue_size=10)
    rospy.init_node('talker1',anonymous=True)
    rate=rospy.Rate(10)
    input_gen=Input()
    while not rospy.is_shutdown():
        keyval=input_gen.send(2)
        keyval=str(keyval)
        rate.sleep()
        #Set Platoon ID
        if(keyval=='a'):
            message="a,1,0,1,2,3,4,5,6,7,8,9"
            #rospy.loginfo(message)
            pub.publish(message)
            rate.sleep()
        #Set robots to follower
        if(keyval=='c'):
            message="c,1,2,0,1,2,3,4,5,6,7,8,9"
            pub.publish(message)
            rate.sleep()
        #Set Lane id
        if(keyval=='d'):
            message="d,1,1,0,1,2,3,4,5,6,7,8,9"
            pub.publish(message)
            rate.sleep()
        #Set platoon leader
        if(keyval=='e'):
            message="e,1,4,6"
            pub.publish(message)
            rate.sleep()
        #Set Speed to leader
        if(keyval=='g'):
            message="g,1,4,65"
            pub.publish(message)
            rate.sleep()
        #Set stop to leader
        if(keyval=='h'):
            message="h,1,4,0"
            pub.publish(message)
            rate.sleep()
        #Set start to leader
        if(keyval=='i'):
            message="i,1,4,1"
            pub.publish(message)
            rate.sleep()
        #Set lane change command to all robots left
        if(keyval=='m'):
            message="m,1,10,0,1,2,3,4,5,6,7,8,9"
            pub.publish(message)
            rate.sleep()
        #Set lane change command to all robots right
        if(keyval=='n'):
            message="n,1,20,0,1,2,3,4,5,6,7,8,9"
            pub.publish(message)
            rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
