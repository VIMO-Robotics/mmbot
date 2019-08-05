#!/usr/bin/env python
'''
  Subscribe to /joy and publish /cmd_vel which make the robot move
'''

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
class JoyMapper(object):
	def __init__(self):
		self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
		self.sub_joy   = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size = 1)
		self.cmd = Twist()
		self.max_vel_x = 0.1
		self.max_vel_theta = 0.6
		rospy.Timer(rospy.Duration(0.1), self.pub_cmd)
	def cb_joy(self,msg):
		self.cmd.linear.x = self.max_vel_x * msg.axes[1]
		self.cmd.angular.z = self.max_vel_theta * msg.axes[3]
	def pub_cmd(self, event):
		self.pub_twist.publish(self.cmd)
	
if __name__ == "__main__":
	rospy.init_node("joy_mapper_node")
	joymapper = JoyMapper()
	rospy.spin()
