#!/usr/bin/env python
import rospy
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseStamped
from robot_navigation.srv import robot_navigation
from math import sqrt
import time

class TagTracker(object):
    def __init__(self):
        # the tag to track
        self.tag_id_track = 1
        # setup service proxy
        rospy.wait_for_service('/robot_navigate')
        self.call_service()
    def call_service(self):
        parking = rospy.ServiceProxy('/robot_navigate', robot_navigation)
        resp = parking(self.tag_id_track)
        print resp.tracking
      
    
if __name__ == "__main__":
    rospy.init_node("tag_tracker",anonymous=False)
    tracker = TagTracker()
    rospy.spin()
