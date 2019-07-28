#!/usr/bin/env python
import rospy
from dagu_car.msg import WheelsCmdStamped, Twist2DStamped
from std_msgs.msg import Float32
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from numpy import *
import rospkg
import yaml
import time
import os.path


# Inverse Kinematics Node
# Author: Robert Katzschmann, Shih-Yuan Liu

class InverseKinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        rospack = rospkg.RosPack()
        self.param_file = rospack.get_path('dagu_car') + "/config/" + "inverse_kinematics.yaml"
        # Set parameters using yaml file
        self.readParamFromFile()

        # Set local variable by reading parameters
        self.gain = self.setup_parameter("~gain", 1.0)
        self.trim = self.setup_parameter("~trim", 0.0)
        self.baseline = self.setup_parameter("~baseline", 0.1)
        self.radius = self.setup_parameter("~radius", 0.0318)
        self.k = self.setup_parameter("~k", 27.0)
        self.limit = self.setup_parameter("~limit", 1.0)

        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("/car_cmd", Twist2DStamped, self.car_cmd_callback, queue_size=1)
        self.sub_trim = rospy.Subscriber("/trim", Float32, self.trimCb, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("/wheels_cmd", WheelsCmdStamped, queue_size=1)
        rospy.loginfo("[%s] Initialized.", self.node_name)
        self.printValues()

    def trimCb(self, msg):
        self.trim = msg.data

    def readParamFromFile(self):
        # Check file existence
        fname = self.param_file
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
            fname = self.getFilePath("default")

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass     

    def printValues(self):
        rospy.loginfo("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))

    def car_cmd_callback(self, msg_car_cmd):
        # assuming same motor constants k for both motors
        k_r = self.k
        k_l = self.k

        # adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l
        
        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        
        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
