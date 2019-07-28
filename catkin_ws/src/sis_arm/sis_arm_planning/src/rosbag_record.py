#!/usr/bin/env python
import rospy
import subprocess as sb
from std_msgs.msg import Bool

rospy.init_node("rosbag_record_node")

def callback(msg):

    if not msg.data:

        sb.call("rosbag record -o ~/isam -e /isam_graph /isam_odom /landmark_graph /landmark_info /measurement_graph /odom /odom_graph /tag_detections __name:=isam",shell=True)

sub_record_ = rospy.Subscriber("/rosbag_record", Bool, callback, queue_size=1)



rospy.spin()
