#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo

rospy.init_node("sync_node")
image_sub = message_filters.Subscriber('/pi_camera/image_rect', Image)
info_sub = message_filters.Subscriber('/pi_camera/camera_info', CameraInfo)

image_pub = rospy.Publisher("/sync/image_rect", Image,  queue_size=1) 
info_pub = rospy.Publisher("/sync/camera_info", CameraInfo,  queue_size=1) 

def callback(image, camera_info):
  image_pub.publish(image)
  info_pub.publish(camera_info)


ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
ts.registerCallback(callback)
rospy.spin()
