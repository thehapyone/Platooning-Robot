#!/usr/bin/env python
import rospy
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing
from bumblee import arduino_in
from threading import Thread

from timeit import default_timer as timer

import sys

finshed = 0
node_time = timer()
status_counter = 0

lcm_started = 0

def main():
    rospy.init_node("Arduino_serial_node", anonymous=False)

    #rospy.init_node("Arduino_serial_node")
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','115200'))

    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)

    # TODO: do we really want command line params in addition to parameter server params?
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2 :
        port_name  = sys.argv[1]
    if len(sys.argv) == 3 :
        tcp_portnum = int(sys.argv[2])

    # Use serial port
    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
        try:
            client = SerialClient(port_name, baud, fix_pyserial_for_test=fix_pyserial_for_test)
            client.run()      
        except KeyboardInterrupt:
            finshed = 1
            exit(1)
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue
    
if __name__=="__main__":
    main()
