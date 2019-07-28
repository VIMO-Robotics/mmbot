#!/usr/bin/env python
import dynamixel_msgs.msg
import sensor_msgs.msg
import message_filters
import rospy
from std_msgs.msg import Float64

rospy.init_node("virtual_command_pub")


joint1_pub = rospy.Publisher("/arm_shoulder_pan_joint/command", Float64,  queue_size=1) 
joint2_pub = rospy.Publisher("/arm_shoulder_lift_joint/command", Float64,  queue_size=1) 
joint3_pub = rospy.Publisher("/arm_elbow_flex_joint/command", Float64,  queue_size=1) 
joint4_pub = rospy.Publisher("/arm_wrist_flex_joint/command", Float64,  queue_size=1) 

def joint_callback(msg):

    j1 = Float64(data = msg.position[2])
    j2 = Float64(data = msg.position[1])
    j3 = Float64(data = msg.position[0])
    j4 = Float64(data = msg.position[3])

    joint1_pub.publish(j1)
    joint2_pub.publish(j2)
    joint3_pub.publish(j3)
    joint4_pub.publish(j4)

command_sub = rospy.Subscriber("/move_group/fake_controller_joint_states", sensor_msgs.msg.JointState, joint_callback, queue_size=1)

rospy.spin()