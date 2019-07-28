#!/usr/bin/env python
import rospy
import numpy as np
import math
from dagu_car.msg import  Twist2DStamped
from std_msgs.msg import Bool
from apriltags2_ros.msg import AprilTagDetectionArray
from sis_arm_msgs.msg import Odom_uno
from geometry_msgs.msg import Point32
from action_server.msg import landmark_info
import tf
from sensor_msgs.msg import Joy
import time
from __builtin__ import True
import subprocess as sb


class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.joy = None
        self.isam_info = None
        self.data_switch = False
        self.log_switch = False
        self.record_switch = 0
        self.landmark_list = [(0,0),(0,0),(0,0),(0,0),(0,0)]
        self.i = 0
        self.j = 0
        self.data_index = 1
        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("/car_cmd", Twist2DStamped, queue_size=1)
        self.pub_record = rospy.Publisher("/rosbag_record", Bool, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_isam_odom = rospy.Subscriber("/isam_odom", Point32, self.cbisam, queue_size=1)
        self.sub_odom = rospy.Subscriber("/odom", Odom_uno, self.cbOdometry, queue_size=1)
        self.sub_apriltags = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cbApriltags, queue_size=1)
        self.sub_landmark_odom = rospy.Subscriber("/landmark_info", landmark_info, self.cbLandmark, queue_size=1)

        with open('/home/ubuntu/log_data.txt', 'w') as file:
          file.write("START TO LOG\n")
        
        # timer
        # self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
    def cbisam(self, msg):
        if not self.data_switch:
            return
        self.isam_info = (msg.x, msg.y, msg.z)

    def cbOdometry(self, msg):
        if not self.data_switch:
            return
        self.odom_info = (msg.odom.x, msg.odom.y, msg.odom.theta)

    def cbApriltags(self, tag_msg):
        if not self.data_switch or len(tag_msg.detections) == 0:
            return

        q = tag_msg.detections[0].pose.pose.pose.orientation
        angle = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.apriltags_info = (tag_msg.detections[0].pose.pose.pose.position.z, - tag_msg.detections[0].pose.pose.pose.position.x, angle[1], tag_msg.detections[0].id[0])

    def cbLandmark(self, msg):
        if not self.data_switch:
            return
        for i in range(len(msg.points)):
          self.landmark_list[msg.ids[i]] = (msg.points[i].x,msg.points[i].y)

    # def cbParamTimer(self,event):
    #     self.v_gain = rospy.get_param("~speed_gain", 1.0)
    #     self.omega_gain = rospy.get_param("~steer_gain", 10)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def data_info(self):
        l0 = "============== Data Info %d ==============\n" %(self.data_index)
        l1 = "odom_info: " + str((np.round(self.odom_info,3))) + "\n"
        l2 = "isam_info: " + str((np.round(self.isam_info,3))) + "\n"
        l3 = "landmark_info: " + str(np.round(self.landmark_list[self.apriltags_info[3]],3)) + "\n"
        error_odom = (self.odom_info[0]- self.landmark_list[self.apriltags_info[3]][0], self.odom_info[1]- self.landmark_list[self.apriltags_info[3]][1])
        error_isam = (self.isam_info[0]- self.landmark_list[self.apriltags_info[3]][0], self.isam_info[1]- self.landmark_list[self.apriltags_info[3]][1])
        # l4 = "error_odom: " + str(np.round(error_odom,3)) + "\n"
        # l5 = "error_isam: " + str(np.round(error_isam,3)) + "\n"
        # l6 = "ground_truth: " + str(np.round(self.apriltags_info[:3],3)) + "\n"
        dist_odom = (error_odom[0] ** 2 + error_odom[1] ** 2) ** 0.5
        dist_isam = (error_isam[0] ** 2 + error_isam[1] ** 2) ** 0.5
        l4 = "dist_odom: " + str(dist_odom) + "\n"
        l5 = "dist_isam: " + str(dist_isam) + "\n"
        dist_groudtruth = (self.apriltags_info[0] ** 2 + self.apriltags_info[1] ** 2) ** 0.5
        l6 = "dist_groudtruth: " + str(dist_groudtruth) + "\n"
        print l0 + l1 + l2 + l3 + l4 + l5 + l6

        if self.log_switch:
          with open('/home/ubuntu/log_data.txt', 'a') as file:
            file.write(l0 + l1 + l2 + l3 + l4 + l5 + l6)

        self.data_index += 1

    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.joy.axes[1] * self.v_gain #Left stick V-axis. Up is positive
        if self.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = self.joy.axes[3] * self.steer_angle_gain
            car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
        self.pub_car_cmd.publish(car_cmd_msg)

# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10

    def processButtons(self, joy_msg):
        if (joy_msg.buttons[4] == 1):
            self.v_gain -= 0.01
            rospy.loginfo("[V_gain] = %f " %(self.v_gain))

        elif (joy_msg.buttons[5] == 1):
            self.v_gain += 0.01
            rospy.loginfo("[V_gain] = %f " %(self.v_gain))

        elif (joy_msg.buttons[6] == 1): #Right back button
            self.omega_gain -= 0.01
            rospy.loginfo("[Omega_gain] = %f " %(self.omega_gain))

        elif (joy_msg.buttons[7] == 1): #the start button
            self.omega_gain += 0.01
            rospy.loginfo("[Omega_gain] = %f " %(self.omega_gain))

        elif (joy_msg.buttons[1] == 1): 
            self.i += 1
            self.data_switch = self.i%2

            if self.data_switch:
                rospy.loginfo("[START TO SUBSCRIBE DATA INFO]")
            else:
                rospy.loginfo("[STOP SUBSCRIBING DATA INFO]")


        elif (joy_msg.buttons[0] == 1): 
            self.j += 1
            self.log_switch = self.j%2

            if self.log_switch:
                rospy.loginfo("[START TO SUBSCRIBE LOG DATA]")
            else:
                rospy.loginfo("[STOP SUBSCRIBING LOG DATA]")


        elif (joy_msg.buttons[3] == 1):
            self.data_info() 


        elif (joy_msg.buttons[2] == 1):

            if self.record_switch:
                sb.call("rosnode kill /isam",shell=True)

                rospy.loginfo("[STOP SUBSCRIBING RECORD ROSBAG]")


            else:
                msg = Bool(data=0)
                self.pub_record.publish(msg)
                rospy.loginfo("[START TO SUBSCRIBE RECORD ROSBAG]")

                self.record_switch += 1

        # elif (joy_msg.buttons[3] == 1):

        # elif (joy_msg.buttons[8] == 1): #power button (middle)

        # elif (joy_msg.buttons[9] == 1): #push left joystick button 

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))
                                          

if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
