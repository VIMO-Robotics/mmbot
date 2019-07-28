#!/usr/bin/env python
import rospy
from dagu_car.msg import WheelsCmdStamped
from dagu_car.dagu_wheels_driver import DaguWheelsDriver

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False

        # Setup publishers
        self.driver = DaguWheelsDriver()
        #add publisher for wheels command wih execution time

        # Setup subscribers
        self.sub_topic = rospy.Subscriber("/wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)

    def cbWheelsCmd(self,msg):
        self.driver.setWheelsSpeed(left=msg.vel_left,right=msg.vel_right)

    def on_shutdown(self):
        self.driver.setWheelsSpeed(left=0.0,right=0.0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
