#!/usr/bin/env python
'''
  Park the robot in front of tag 1 directly from map coordinate 
'''
import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	rospy.init_node("parking_node")
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
	listener = tf.TransformListener()
	state = True
	while state is True:
		# Listen to the transform from map to car_base
		try:
			(trans, rot) = listener.lookupTransform('map', 'car_base', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		# Since the move_base node will not output reverse behavior, if the robot keep move
		# forward it will collide the platform, we directly publish negative linear velocity
		# to reverse the robot until the component x of translation from map to car_base
		# reach 1 (about the line from tag 4 to tag 6)
		if trans[0] < 1:
			state = True
			cmd = Twist()
			cmd.linear.x = -0.08
			pub_cmd.publish(cmd)
			print "Pub cmd"
		else:
			# Break while loop
			state = False 
			print "X reach 1, start using move_base action"
	# Start using move_base action, first waiting for server
	client.wait_for_server()
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map" # W.r.t. 'map' frame
	goal.target_pose.header.stamp = rospy.Time.now() # Request time is 'now'
	goal.target_pose.pose.position.x = 2.47 # Tag 1 parking lot position x w.r.t. map frame
	goal.target_pose.pose.position.y = 0.9  # Tag 1 parking lot position y w.r.t. map frame
	goal.target_pose.pose.orientation.w = 1.0 # Fase toward tag
	client.send_goal(goal)
	client.wait_for_result()
	print "Finish"
	# Since we don't use the detection of tag as feedback, you may notice that the vehicle 
	# not park very will
