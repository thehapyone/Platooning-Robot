#!/usr/bin/env python

# importing the ros python client library. 
# rospy is needed for the ROS node. std_msgs.msg is so that we can reusue the std_msg/String
# message type
import rospy
from std_msgs.msg import String

def talker():
	# initializing the publisher. Here 'chatter' is the topic to publish to.
	pub = rospy.Publisher('chatter', String, queue_size=10)
	# create the ROS node and give it a name
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) #10hz

	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		# publish the message
		pub.publish(hello_str)
		rate.sleep()

if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
