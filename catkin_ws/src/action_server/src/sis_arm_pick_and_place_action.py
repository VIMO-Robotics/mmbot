#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float64, Bool
from action_server.msg import block_pick_and_placeAction, block_pick_and_placeResult, block_pick_and_placeFeedback
import actionlib
import tf
from ik_4dof import ik_solver
import numpy as np

class block_pick_and_place(object):
  def __init__(self):
      self.node_name = rospy.get_name() 
      # Initial
      self.gripper_v = 1.23 # for cube
      self.br = tf.TransformBroadcaster()
      self.tf_listener = tf.TransformListener()
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

      self.pub_current_joint_state = rospy.Publisher('/pub_current_joint_state', Bool, queue_size=1)

      self._as = actionlib.SimpleActionServer('block_pick_and_place_server', block_pick_and_placeAction, execute_cb=self.execute_cb, auto_start = False)
      self._feedback = block_pick_and_placeFeedback()
      self._result = block_pick_and_placeResult()
      self._as.start()

      # safe shutdown
      rospy.on_shutdown(self.onShutdown)

      rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
      self.group.allow_replanning(True)
      self.group.set_pose_reference_frame("base_link")

      self.pub_current_joints()
      self.gripper_action(0)
      self.home_pose()

  def pub_current_joints(self):
      msg = Bool(data=1)
      self.pub_current_joint_state.publish(msg)
      rospy.sleep(1)
      
      msg.data = 0
      self.pub_current_joint_state.publish(msg)

  def execute_cb(self, goal):
      rospy.loginfo("Goal Received !")
      self.gripper_v = goal.object_size

      if goal.mode == 0 or goal.mode == 1:
        self.ready_pose(1)
        print "========== ready_pose finished ==========\n"
        self.stack_mode(goal.pick_pose, goal.place_pose)

      elif goal.mode == 2:
        self.classifier_mode(goal.pick_pose, goal.place_pose)

      self.home_pose()
      self._result.state = True
      self._as.set_succeeded(self._result)
      rospy.loginfo('block_pick_and_place: Succeeded')

  def stack_mode(self,pick_pose, place_pose):
      for i in range(len(pick_pose)):
        print "========== pick_pose =========="
        print [pick_pose[i].pose.position.x, pick_pose[i].pose.position.y, pick_pose[i].pose.position.z]
        print "========== place_pose =========="
        print [place_pose[i].pose.position.x, place_pose[i].pose.position.y, place_pose[i].pose.position.z]

        self.pre_action_pose(copy.deepcopy(pick_pose[i]))
        print "========== pre_action_pose finished =========="

        self.action_pose(pick_pose[i])
        print "========== Pick finished =========="

        self.gripper_action(1)
        print "========== grasp cube finished =========="

        self.pre_action_pose(copy.deepcopy(pick_pose[i]))
        print "========== pre_action_pose finished =========="

        self.ready_pose(0)
        print "========== ready_pose finished =========="

        self.pre_action_pose(copy.deepcopy(place_pose[i]))
        print "========== pre_action_pose finished =========="

        self.action_pose(place_pose[i])
        print "========== Place finished =========="

        self.gripper_action(0)
        print "========== Place cube finished =========="

        self.pre_action_pose(copy.deepcopy(place_pose[i]))
        print "========== pre_action_pose finished =========="

        self.ready_pose(0)
        print "========== ready_pose finished ==========\n"

  def classifier_mode(self,pick_pose, place_pose):
      for i in range(len(pick_pose)):
        print "========== pick_pose =========="
        print [pick_pose[i].pose.position.x, pick_pose[i].pose.position.y, pick_pose[i].pose.position.z]
        print "========== place_pose =========="
        print [place_pose[i].pose.position.x, place_pose[i].pose.position.y, place_pose[i].pose.position.z]

        self.pre_action_pose(copy.deepcopy(pick_pose[i]))
        print "========== pre_action_pose finished =========="

        self.action_pose(pick_pose[i])
        print "========== Pick finished =========="

        self.gripper_action(1)
        print "========== grasp cube finished =========="

        self.pre_action_pose(copy.deepcopy(pick_pose[i]))
        print "========== pre_action_pose finished =========="

        self.ready_pose(0)
        print "========== ready_pose finished =========="

        self.pre_action_pose(copy.deepcopy(place_pose[i]))
        print "========== pre_action_pose finished =========="

        self.action_pose(place_pose[i])
        print "========== Place finished =========="

        self.gripper_action(0)
        print "========== Place cube finished =========="

        self.pre_action_pose(copy.deepcopy(place_pose[i]))
        print "========== pre_action_pose finished =========="

        self.ready_pose(2)
        print "========== ready_pose finished ==========\n"

  def home_pose(self):
      self.execute_fk(0.7976700097005335, -2.1782527187976104, 2.188479257383515, 0.4601942363656924)
      self.execute_fk(-0.02045307717180855, -2.162912910918754, 2.0657607943526637, 0.8)

  def ready_pose(self, state):
      if state == 0:
        self.execute_fk(0,-0.3681553890925539,1.6055665579869711,1.6413594430376361)

      elif state == 1:
        # self.execute_fk(0.8130098175793898, -2.0197413707160945, 2.1782527187976104, 0.3425890426277932)
        self.execute_fk(0.7976700097005335, -2.1782527187976104, 2.188479257383515, 0.4601942363656924)
        self.execute_fk(0,-0.3681553890925539,1.6055665579869711,1.6413594430376361)

      elif state == 2:
        self.execute_fk(0,-0.3681553890925539,1.6055665579869711,1.6413594430376361)
        self.execute_fk(0.7976700097005335, -2.1782527187976104, 2.188479257383515, 0.4601942363656924)

      # rospy.sleep(1.5)

  def pre_action_pose(self, pre_pose):
      pre_pose.pose.position.z += 0.02
      # print [pre_pose.pose.position.x, pre_pose.pose.position.y, pre_pose.pose.position.z]
      self.find_ik_and_execute(pre_pose)
      # rospy.sleep(1.5)

  def action_pose(self, pose):
      # print [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
      self.find_ik_and_execute(pose)

  def execute_fk(self, theta_1, theta_2, theta_3, theta_4):
      if rospy.is_shutdown():
        rospy.loginfo('%s: Finished' % self._action_name)
        self._as.set_preempted()

      joint_values = self.group.get_current_joint_values()

      joint_values[0] = theta_1
      joint_values[1] = theta_2
      joint_values[2] = theta_3
      joint_values[3] = theta_4

      self.group.set_joint_value_target(joint_values)
      plan = self.group.plan()
      return self.group.execute(plan,wait=True)

  def gripper_action(self, state):
      if state:
        msg = Float64(data=self.gripper_v) #1.23
        s_t = 1.5
      else:
        msg = Float64(data=0)
        s_t = 2

      self.gripper_cmd.publish(msg)
      rospy.sleep(s_t)

  def find_ik_and_execute(self, pose_transformed):
      x = pose_transformed.pose.position.x 
      y = pose_transformed.pose.position.y 
      z = pose_transformed.pose.position.z

      ik_candidate = ik_solver(x, y, z, -90)
      # print "========== Find ",len(ik_candidate)," Plan =========="
      if not np.isnan(ik_candidate.all()):
        for theta_1, theta_2, theta_3, theta_4 in ik_candidate:
          # while not rospy.is_shutdown():
          try:
            if self.execute_fk(theta_1, theta_2, theta_3, theta_4):
              # rospy.loginfo("========== Execute Plan ==========")
              # print [theta_1, theta_2, theta_3, theta_4]
              break
          except Exception as e:
              # rospy.loginfo(e)
              # print "------------- Failed -------------"
              # print [theta_1, theta_2, theta_3, theta_4],"\n"
              continue

      else:
        rospy.loginfo("========== Cannot Find Solution ==========")
        self._result.state = False
        self._as.set_aborted(self._result)
        

  def onShutdown(self):
      rospy.loginfo("[%s] Shutting down..." %self.node_name)
      rospy.sleep(0.5) #To make sure that it gets published.
      rospy.loginfo("[%s] Shutdown" %self.node_name)


if __name__ == '__main__': 
    rospy.init_node('block_pick_and_place',anonymous=False)
    block_pick_and_place = block_pick_and_place()
    rospy.on_shutdown(block_pick_and_place.onShutdown)
    rospy.spin()
