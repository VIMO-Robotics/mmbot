#!/usr/bin/env python

'''
  Given present pose [x, y, th] of car w.r.t map and target tag ID from client,
  first reach the temporary point, which in front of the tag with distance 
  @self.dist_from_park, and then revise the heading to toward the tag, and 
  move to the parking lot.
  
  Editor: Sean, Allen
  Last modify: 1/20, 2018
'''

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_navigation.srv import *
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from math import pi, atan2, sqrt

class Parking(object):
	def __init__(self):
		# Parking lot map
		self.park_x = {'0':2.47, '1':2.47, '2':2.47, '5':0.7} 
        	self.park_y = {'0':0.30, '1':0.90, '2':1.50, '5':0.9}
        	self.park_th = {'0':0, '1':0, '2':0, '5':pi}
                self.dist_from_park = 0.2
			
		# Threshold
		self.dist_thres = 0.05 # 0.05 meter in front of tag
		self.head_thres = 0.1 # 0.1 rad around tag
		self.v_thres = 0.05 # Linear velocity
		self.w_thres = 0.12 # Angular velocity

		self.listener = tf.TransformListener()
		self.sign = 0 # State sign, 0 means at temporary point, 1 means complete the task

		# Publisher & Service
		self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self.srv_navigate = rospy.Service("/robot_navigate", robot_navigation, self.cbNavigate)

	def cbNavigate(self, req):
		if self.park_x.get(str(req.id)) == None:
			res = robot_navigationResponse()
			res.tracking = "NotSupportID"
			res.at_id = -1
			return res
		target_id = req.id # Get target tag id from client
		x_d = self.park_x.get(str(target_id))
		y_d = self.park_y.get(str(target_id))
		th_d = self.park_th.get(str(target_id))
		# Arrive temporary point first
		if target_id == 5:
			x_d += self.dist_from_park
		else:
			x_d -= self.dist_from_park
		while not rospy.is_shutdown():
			try:
				listener = tf.TransformListener()
				listener.waitForTransform("map", "car_base", rospy.Time(0), rospy.Duration(3.0))
				(trans, rot) = listener.lookupTransform("map", "car_base", rospy.Time(0))
			except:
				print "Exception"
				continue
			x = trans[0]
			y = trans[1]
                   	quat = (rot[0], rot[1], rot[2], rot[3])
			euler = tf.transformations.euler_from_quaternion(quat)
			th = euler[2]
			print "Now at: " + str(x) + ' ' + str(y) + ' ' + str(th)
			heading = atan2(y_d - y, x_d - x)
			print "Heading: " + str(heading)
			th_diff = heading - th
			# Make sure rotate in excellent arc
			if abs(th_diff) > pi:
				if th_diff > 0:
					th_diff = -(2*pi - th_diff)
				else:
					th_diff = (2*pi + th_diff)
			print "th_diff:", th_diff
			# Heading not in threshold
			if abs(th_diff) > self.head_thres:
				time = abs(th_diff) / self.w_thres
				cmd = Twist()
				print "Rotate %f seconds" %(time)
				cmd.angular.z = th_diff/abs(th_diff) * self.w_thres
				self.pub_cmd.publish(cmd)
				rospy.sleep(time)
				print "Stop rotating"
				cmd = Twist()
				self.pub_cmd.publish(cmd)
				continue
			dist = sqrt((x_d - x)**2 + (y_d - y)**2)
			print "Distance: ", dist
			# Distance not in threshold
			if dist > self.dist_thres:
				time = dist / self.v_thres
				print "Need to go straight %f seconds" %(time)
				cmd = Twist()
				cmd.linear.x = self.v_thres
				self.pub_cmd.publish(cmd)
				rospy.sleep(time/2)
				cmd = Twist()
        self.pub_cmd.publish(cmd)
			elif dist <= self.dist_thres:
				if self.sign == 0:
					print "reach waypoint"
					self.sign = 1
					x_d = self.park_x.get(str(target_id)) # Set x_d to parking lot x
				else:
					print "reach final goal"
					break
				continue
		print "Complete"
		res = robot_navigationResponse()
		res.tracking = "Complete"
		res.at_id = target_id
		return res

if __name__ == "__main__":
	rospy.init_node("parking_node")
	parking = Parking()
	rospy.spin()
