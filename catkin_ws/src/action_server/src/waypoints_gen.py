#!/usr/bin/env python
import rospy
from action_server.msg import path_followingArrayAction, path_followingArrayGoal, Point2D
import actionlib
import numpy as np
from std_msgs.msg import Int16

class WaypointsGen(object):
	def __init__(self):
		self.node_name = rospy.get_name()

		# subsrciber
		self.gen_sub = rospy.Subscriber('/waypoints_gen', Int16, self.genCb, queue_size=1)
		self.client = actionlib.SimpleActionClient('path_following_action', path_followingArrayAction)
		self.client.wait_for_server()
		rospy.loginfo('================= Find path_following_action =================')

		self.waypoints = []
		self.count = 0


		rospy.on_shutdown(self.custom_shutdown) # shutdown method
		rospy.loginfo("[%s] Initialized " %self.node_name)

	def genCb(self, msg):
		if msg.data == 1:
			self.waypoints = [[(0,0),(2.4,0),(2.4,1.8), (0,1.8), (0,0.2)],[(0,0.4), (0,0), (0.2,0)]]
			# waypoints = [(0,0),(3,0),(3,2.4),(1.5,2.4)]
		# elif msg.data == 2:
			# waypoints = [(0,0.4), (0,0), (0.2,0)]
			# waypoints = [(1.7,2.4), (0,2.4), (0.2,0)]
		self.count = 0

		self.send_waypoints_goal(self.waypoints[self.count])

	def send_waypoints_goal(self, waypoints):
		waypoints_list = []
		for i in range((len(waypoints)-1)):
			waypoints_list += (zip(np.linspace(waypoints[i][0], waypoints[i+1][0], 20, endpoint=False), np.linspace(waypoints[i][1], waypoints[i+1][1], 20, endpoint=False)))
		waypoints_list.append(waypoints[-1])

		goal=path_followingArrayGoal()

		for x,y in waypoints_list:
			# print x,y
			waypoint = Point2D()
			waypoint.x = x
			waypoint.y = y
			goal.waypoints.append(waypoint)

		self.client.send_goal(goal,feedback_cb=self.distCb)

		# print waypoints_list
		# print len(waypoints_list)

	def distCb(self, msg):
		if msg.dist < 0.05:
			self.count += 1
			self.send_waypoints_goal(waypoints[self.count]%2)

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)

if __name__ == "__main__":
	rospy.init_node("waypoints_gen", anonymous = False)
	waypoints_gen_node = WaypointsGen()
	rospy.spin()

