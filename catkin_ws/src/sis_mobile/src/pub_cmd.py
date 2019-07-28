#!/usr/bin/env python

# Test if the controller can reach the given value
# arg: linear.x angular.z

import sys
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	if len(sys.argv)<= 3:
		print "Not enough arguments given, please give velocity and angular velocity!"
		sys.exit(0)
        rospy.init_node("pub_cmd_node")
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        twist = Twist()
        twist.linear.x = float(sys.argv[1])
        twist.angular.z = float(sys.argv[2])
        r = rospy.Rate(100) # Publish with 100Hz
        while not rospy.is_shutdown():
                pub.publish(twist)
                r.sleep()
