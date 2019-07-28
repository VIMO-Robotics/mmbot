#!/usr/bin/env python
import dynamixel_msgs.msg
import sensor_msgs.msg
import message_filters
import rospy
from std_msgs.msg import Bool


rospy.init_node("virtual_joint_pub")
joint1_sub = message_filters.Subscriber("/arm_shoulder_pan_joint/state", dynamixel_msgs.msg.JointState)
joint2_sub = message_filters.Subscriber("/arm_shoulder_lift_joint/state", dynamixel_msgs.msg.JointState)
joint3_sub = message_filters.Subscriber("/arm_elbow_flex_joint/state", dynamixel_msgs.msg.JointState)
joint4_sub = message_filters.Subscriber("/arm_wrist_flex_joint/state", dynamixel_msgs.msg.JointState)
joint5_sub = message_filters.Subscriber("/gripper_joint/state", dynamixel_msgs.msg.JointState)

joint_pub = rospy.Publisher("/move_group/fake_controller_joint_states", sensor_msgs.msg.JointState,  queue_size=1) 


joint5 = 0
switch = False

def joint5Cb(msg):
    global joint5
    joint5 = msg.position[4]

def pub_joint_subCb(msg):
    global switch
    switch = msg.data


gripper_link_joint_sub = rospy.Subscriber("/move_group/fake_controller_joint_states", sensor_msgs.msg.JointState, joint5Cb, queue_size=1)
pub_joint_sub = rospy.Subscriber("/pub_current_joint_state", Bool, pub_joint_subCb, queue_size=1)


    

def joint_callback(j1, j2, j3, j4, j5):
    global switch
    global joint5

    if not switch:
        return

    # print joint5
    #print j1, j2
    js = sensor_msgs.msg.JointState( header = j1.header, 
                                     name = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'gripper_link_joint'],
                                     position = [j1.current_pos, j2.current_pos, j3.current_pos, j4.current_pos, joint5],
                                     velocity = [j1.velocity, j2.velocity, j3.velocity, j4.velocity, j5.velocity],
                                     effort = [j1.load, j2.load, j3.load, j4.load, j5.load] )
    joint_pub.publish(js)
    print "Initial joint state"
ts = message_filters.ApproximateTimeSynchronizer([joint1_sub, joint2_sub, joint3_sub, joint4_sub, joint5_sub], 10, 1)
ts.registerCallback(joint_callback)

rospy.spin()
