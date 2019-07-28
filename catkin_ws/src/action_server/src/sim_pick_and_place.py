#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Twist, PoseStamped, PointStamped
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Int32, Bool, Float64
import time
import math
import threading
import random
from action_server.msg import block_pick_and_placeAction, block_pick_and_placeResult, block_pick_and_placeFeedback
import actionlib
from sis_arm_msgs.msg import ObjectTracking
import tf
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QWidget
from ik_4dof import ik_solver
import numpy as np

class block_pick_and_place(QWidget):
  def __init__(self, title="block_pick_and_place"):
      super(block_pick_and_place, self).__init__()

      # Initial
      self.br = tf.TransformBroadcaster()
      self.tf_listener = tf.TransformListener()
      self.ik = [0,0,0,0]
      # Thread lock 

      moveit_commander.roscpp_initialize(sys.argv)
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander("arm")
   
      display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=1)
      self.gripper_cmd = rospy.Publisher('/gripper_joint/command', Float64, queue_size=1)

      # safe shutdown
      rospy.on_shutdown(self.onShutdown)

      # timer
      rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
      self.group.allow_replanning(True)
      self.group.set_pose_reference_frame("base_link")


      self.group.set_goal_position_tolerance(0.005)
      self.group.set_goal_orientation_tolerance(0.05)

      self.home_pose()
      self.init_interactive_marker()

      self.arm_plan_state = False


      self.vlayout = QVBoxLayout(self)
      self.gridlayout = QGridLayout()
      self.vlayout.addLayout(self.gridlayout)

      # Buttons for randomizing and centering sliders and
      # Spinbox for on-the-fly selecting number of rows

      self.CWbutton = QPushButton('Start_to_plan', self)
      self.CWbutton.clicked.connect(self.start_event)
      self.vlayout.addWidget(self.CWbutton)
      self.SPbutton = QPushButton('Stop', self)
      self.SPbutton.clicked.connect(self.stop_event)
      self.vlayout.addWidget(self.SPbutton)

  def start_event(self, event):
      rospy.loginfo('ARM: START')
      self.arm_plan_state = True
      self.start_ik(self.ik[0], self.ik[1], self.ik[2], self.ik[3])

  def stop_event(self, event):
      rospy.loginfo('ARM: STOP ')
  
      self.arm_plan_state = False
      # print self.waypoints
      

  def init_interactive_marker(self):
      server = InteractiveMarkerServer("simple_marker")
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = "/world"
      int_marker.name = "Object"
      int_marker.description = "Object Control"
      int_marker.scale = 0.1
      int_marker.pose.position.x = 0.3
      int_marker.pose.position.z = 0.01
          # create a grey box marker
      box_marker = Marker()
      box_marker.type = Marker.CUBE
      box_marker.scale.x = 0.02
      box_marker.scale.y = 0.02
      box_marker.scale.z = 0.02
      box_marker.color.r = 0.0
      box_marker.color.g = 0.5
      box_marker.color.b = 0.5
      box_marker.color.a = 1.0

      # create a non-interactive control which contains the box
      box_control = InteractiveMarkerControl()
      box_control.always_visible = True
      box_control.markers.append( box_marker )

      # add the control to the interactive marker
      int_marker.controls.append( box_control )

      # create a control which will move the box
      # this control does not contain any markers,
      # which will cause RViz to insert two arrows
      rotate_control = InteractiveMarkerControl()
      rotate_control.name = "move_3D"
      rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_3D

      # add the control to the interactive marker
      int_marker.controls.append(rotate_control);

      # add the interactive marker to our collection &
      # tell the server to call processFeedback() when feedback arrives for it
      control = InteractiveMarkerControl()


      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 1
      control.orientation.y = 0
      control.orientation.z = 0
      control.name = "move_x"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 0
      control.orientation.y = 1
      control.orientation.z = 0
      control.name = "move_z"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)


      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 0
      control.orientation.y = 0
      control.orientation.z = 1
      control.name = "move_y"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)





      server.insert(int_marker, self.processFeedback)

      # 'commit' changes and send to all clients
      server.applyChanges()

  def processFeedback(self, feedback):
      p = feedback.pose.position
      self.tf_listener.waitForTransform("/base_link", "/world", rospy.Time(0),rospy.Duration(1))
      pp = PointStamped()
      pp.point = p
      pp.header.frame_id = "/world"
      point_after = self.tf_listener.transformPoint("arm_base_link",pp)
      x = point_after.point.x #+ random.uniform(0.005, -0.005)
      y = point_after.point.y #+ random.uniform(0.005, -0.005)
      # print "x: ",x, "  y: ",y
      z = point_after.point.z
      ik_candidate = ik_solver(x, y, z, -90)
      # print ik_candidate
      if not np.isnan(ik_candidate.all()):
        for theta_1, theta_2, theta_3, theta_4 in ik_candidate:
          if abs(theta_2) < 2 and abs(theta_3) < 2 and abs(theta_4) < 2:
            self.ik = [theta_1, theta_2, theta_3, theta_4]
            break


      # print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

  def home_pose(self):
      joint_values = self.group.get_current_joint_values()


      joint_values[0] = 0
      joint_values[1] = -1.2417246372984723
      joint_values[2] = 1.7997449697893113
      joint_values[3] = 1.4184705050066115

      self.group.set_joint_value_target(joint_values)
      plan2 = self.group.plan()
      self.group.execute(plan2,wait=True) 

  def start_ik(self, theta_1, theta_2, theta_3, theta_4):
      joint_values = self.group.get_current_joint_values()

      joint_values[0] = theta_1
      joint_values[1] = theta_2
      joint_values[2] = theta_3
      joint_values[3] = theta_4

      self.group.set_joint_value_target(joint_values)
      plan2 = self.group.plan()

      return plan2
      # self.group.execute(plan2,wait=True) 


  def planCB(self, point_before, switch):
      # print msg

      self.tf_listener.waitForTransform("/base_link", "/world", rospy.Time(0),rospy.Duration(1))
 
      point_after = self.tf_listener.transformPoint("base_link",point_before)
      x = point_after.point.x #+ random.uniform(0.005, -0.005)
      y = point_after.point.y #+ random.uniform(0.005, -0.005)
      # print "x: ",x, "  y: ",y
      z = point_after.point.z

      # print "x: ",x, "  y: ",y," z: ",z
      pose_target = Pose()
      pose_target.position.x = x
      pose_target.position.y = y
      pose_target.position.z = z
      r = math.atan(y/x)
      p = math.radians(90)
      y = 0
      if switch:
        self.rotation(r)
    # while (1):

      # q = tf.transformations.quaternion_from_euler(math.radians(msg.data[3]), math.radians(msg.data[4]), math.radians(msg.data[5]))
      q = tf.transformations.quaternion_from_euler(r, p, y)
      pose_target.orientation.x = q[0]
      pose_target.orientation.y = q[1]
      pose_target.orientation.z = q[2]
      pose_target.orientation.w = q[3]
      # print pose_target

      # self.group.set_goal_tolerance(0.1)
      self.group.set_pose_target(pose_target)
      # self.group.set_position_target((msg.data[0],msg.data[1],msg.data[2]))
      self.plan = self.group.plan()
      # print self.group.plan()
      ok = self.group.execute(self.plan, wait=True)
      print ok
      return ok
      # if ok:
      #   break
      # # print self.plan
      # p += 0.01 
      # print p
       

  def onShutdown(self):
      rospy.loginfo("[%s] Shutting down..." %self.node_name)
      rospy.sleep(0.5) #To make sure that it gets published.
      rospy.loginfo("[%s] Shutdown" %self.node_name)


if __name__ == '__main__': 
    rospy.init_node('block_pick_and_place',anonymous=False)
    app = QApplication(sys.argv)
    block_pick_and_place = block_pick_and_place()
    block_pick_and_place.show()
    app.exec_()
    rospy.on_shutdown(block_pick_and_place.onShutdown)
    rospy.spin()
