#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sis_arm_msgs.msg import Odom_uno
from action_server.msg import mm_taskAction, mm_taskFeedback, mm_taskResult, Point2D
import actionlib
import numpy as np
import math

class mm_task_planning_node():
    def __init__(self):


        self.landmark_list = [(0,0), (0,0), (0,0), (0,0), (0,0)]

        self.sub_isam_odom = rospy.Subscriber("/isam_odom", Point32, self.cbOdometry, queue_size=1)
        self.sub_landmark_odom = rospy.Subscriber("/landmark_info", landmark_info, self.cbLandmark, queue_size=1)
        self.sub_apriltags = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cbApriltags, queue_size=1)

        self.mm_task_planning_action = actionlib.SimpleActionServer("mm_task_planning_node", mm_taskAction, execute_cb=None, auto_start = False)
        self.mm_task_planning_action.register_goal_callback(self.actionCB)
        self.mm_task_feedback = mm_taskFeedback()
        self.mm_task_result = mm_taskResult()

        self.mm_task_planning_action.start()

        rospy.loginfo('mm_task_planning_node: Initialized')


    def cbLandmark(self, msg):
        if self.action_state != 1:
            for i in range(len(msg.points)):
                self.landmark_list[msg.ids[i]] = (msg.points[i].x,msg.points[i].y)


    def actionCB(self):
        goal = self.mm_task_planning_action.accept_new_goal()


    def action_succeeded(self):
        rospy.loginfo('mm_task: Succeeded')
        self.mm_task_planning_action.set_succeeded(self.mm_task_result)


    def cbOdometry(self, msg):
        if self.action_state == 1:
            self.robot_pose = (msg.odom.x, msg.odom.y, msg.odom.theta)
        else:
            self.robot_pose = (msg.x, msg.y, msg.z)


        if not self.mm_task_planning_action.is_active():
            return 

        self.start_pure_pursuit()
        self.mm_task_feedback.dist = self.getDistance(self.robot_pose, self.waypoints[-1])
        self.mm_task_planning_action.publish_feedback(self.mm_task_feedback)




if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("mm_task_planning_node",anonymous=False)
    mm_task_planning_node = mm_task_planning_node()
    rospy.spin()
