#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point32, PoseStamped, Point
from sis_arm_msgs.msg import Odom_uno
from visualization_msgs.msg import Marker
from action_server.msg import landmark_info
import math



class PubFrame(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.br = tf.TransformBroadcaster()
		self.pub_state = rospy.get_param("~pub_state", 2)

		# subsrciber
		self.sub_isam_odom = rospy.Subscriber("/isam_odom", Point32, self.cbCorrect, queue_size=1)
		self.sub_landmark_odom = rospy.Subscriber("/landmark_info", landmark_info, self.cbLandmark, queue_size=1)
		self.sub_odom = rospy.Subscriber("/odom", Odom_uno, self.cbodom, queue_size=1)
		self.sub_tango = rospy.Subscriber("/tango_sensors/tango_pose", PoseStamped, self.cbtango, queue_size=1)
		self.pub_tango_odom = rospy.Publisher("/tango_odom", Marker, queue_size=1)

		self.tango_pose_marker = Marker()
		self.world_tf_offset = 0
		rospy.on_shutdown(self.custom_shutdown) # shutdown method
		rospy.loginfo("[%s] Initialized " %self.node_name)

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)

	def cbCorrect(self, msg):
		if self.pub_state == 1 or self.pub_state == 3:
			self.br.sendTransform((msg.x + self.world_tf_offset, msg.y, 0), # to 3d translation
	                          tf.transformations.quaternion_from_euler(0, 0, msg.z), # to 3d rotation
	                          rospy.Time.now(), # timestamp
	                          "car_base", # robot frame
	                          "world") # base frame

	def cbLandmark(self, msg):
		if self.pub_state == 1 or self.pub_state == 3:
			# rot = [math.pi/2, 0, -math.pi/2, -math.pi/2, math.pi]
			rot = [math.pi/2, math.pi, -math.pi/2, -math.pi/2, 0]
			for i in range(len(msg.points)):
				self.br.sendTransform((msg.points[i].x, msg.points[i].y, 0), # to 3d translation
	                      tf.transformations.quaternion_from_euler(0, 0, rot[msg.ids[i]]), # to 3d rotation
	                      rospy.Time.now(), # timestamp
	                      "tag_id_" + str(msg.ids[i]), # robot frame
	                      "world") # base frame

	def cbodom(self, msg):
		if self.pub_state == 2:
			self.br.sendTransform((msg.odom.x + self.world_tf_offset, msg.odom.y, 0), # to 3d translation
	                          tf.transformations.quaternion_from_euler(0, 0, msg.odom.theta), # to 3d rotation
	                          rospy.Time.now(), # timestamp
	                          "car_base", # robot frame
	                          "world") # base frame

	def cbtango(self, msg):
		if self.pub_state == 3:
			self.tango_pose_marker.header.frame_id = "world"
			self.tango_pose_marker.header.stamp = rospy.Time.now()
			self.tango_pose_marker.action = 0
			self.tango_pose_marker.id = 0
			self.tango_pose_marker.type = 4
			self.tango_pose_marker.pose.orientation.w = 1
			self.tango_pose_marker.scale.x = self.tango_pose_marker.scale.y = self.tango_pose_marker.scale.z = 0.01
			self.tango_pose_marker.color.a = self.tango_pose_marker.color.r = self.tango_pose_marker.color.b = 1
			tango_point = Point(x=msg.pose.position.x, y=msg.pose.position.y)
			self.tango_pose_marker.points.append(tango_point)

			self.pub_tango_odom.publish(self.tango_pose_marker)


if __name__ == "__main__":
	rospy.init_node("pub_frame", anonymous = False)
	pub_frame__node = PubFrame()
	rospy.spin()
