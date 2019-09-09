#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from __builtin__ import True

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

        # timer
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)

    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        car_cmd_msg = Twist()
        # scar_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.linear.x = self.joy.axes[1] * self.v_gain #Left stick V-axis. Up is positive
        if self.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = self.joy.axes[3] * self.steer_angle_gain
            car_cmd_msg.angular.z = car_cmd_msg.linear.x / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.angular.z = self.joy.axes[3] * self.omega_gain

        print car_cmd_msg
        self.pub_car_cmd.publish(car_cmd_msg)

# Button List index of joy.buttons array:
# 0: A 
# 1: B 
# 2: X 
# 3: Y 
# 4: Left Back 
# 5: Right Back
# 6: Back
# 7: Start
# 8: Logitek 
# 9: Left joystick
# 10: Right joystick

# XXX: here we should use constants
    def processButtons(self, joy_msg):
        # if (joy_msg.buttons[x] == 1):

        return
if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
