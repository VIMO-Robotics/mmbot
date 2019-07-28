#!/usr/bin/env python
import rospy
from dagu_car.msg import Twist2DStamped
from geometry_msgs.msg import Twist



class car_control(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        # subsrciber
        self.sub_odom = rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd, queue_size=1)
        # publisher 
        self.pub_car_cmd = rospy.Publisher("/car_cmd", Twist2DStamped, queue_size=1)

        rospy.on_shutdown(self.custom_shutdown) # shutdown method
        rospy.loginfo("[%s] Initialized " %self.node_name)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)


    def cb_cmd(self, msg):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = msg.linear.x
        car_cmd_msg.omega = msg.angular.z
        self.pub_car_cmd.publish(car_cmd_msg)

	

if __name__ == "__main__":
    rospy.init_node("car_control", anonymous = False)
    car_control_node = car_control()
    rospy.spin()
