#!/usr/bin/env python
import sys
import copy
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64
from action_server.msg import block_pick_and_placeAction, block_pick_and_placeGoal, block_detectionAction, block_detectionGoal,block_taskAction, block_taskResult, block_taskFeedback
import actionlib
import tf
import numpy as np

class pick_and_place_server(object):
  def __init__(self):
      self.node_name = rospy.get_name() 
      self.tf_listener = tf.TransformListener()
      self.gripper_size = [1.86, 1.55, 1.22, 0.95]


      self.client_1 = actionlib.SimpleActionClient('cylinder_detection', block_detectionAction)
      self.client_1.wait_for_server()
      self.client_2 = actionlib.SimpleActionClient('block_pick_and_place_server', block_pick_and_placeAction)
      self.client_2.wait_for_server()

      self._as = actionlib.SimpleActionServer('pick_and_place_server', block_taskAction, execute_cb=self.execute_cb, auto_start = False)
      self._feedback = block_taskFeedback()
      self._result = block_taskResult()
      self._as.start()


      # safe shutdown
      rospy.on_shutdown(self.onShutdown)
      rospy.loginfo("[%s] Initialized " %(rospy.get_name()))


  def execute_cb(self, goal):
      rospy.loginfo("[%s] Received goal! " %(rospy.get_name()))

      if goal.mode == 0 or goal.mode == 1:
        self.stack_mode(goal)


      elif goal.mode == 2:
        self.classifier_mode(goal)


      self._result.state = True
      self._as.set_succeeded(self._result)
      rospy.loginfo('block_pick_and_place: Succeeded')

  def pose_transform(self, pose_before):
      while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform("/base_link", "/world", rospy.Time.now(),rospy.Duration(1))
                return self.tf_listener.transformPose("base_link",pose_before)
            except Exception as e:
                rospy.loginfo(e)
                pose_before.header.stamp = rospy.Time.now()
                continue

  def place_pose_gen(self, index):
      place_pose_list = [[0.0862, -0.12, -0.05],[0.1441, -0.07055, -0.052],[0.1515, -0.0029, -0.052],[0.1363, 0.0634, -0.052],[0.0862, 0.1054, -0.052]]
      place_pose = Pose()
      place_pose.orientation.w = 1
      place_pose.position.x = place_pose_list[index][0]
      place_pose.position.y = place_pose_list[index][1]
      place_pose.position.z = place_pose_list[index][2]
      return place_pose


  def stack_mode(self,goal):
      rospy.loginfo("========== Start Stack Mode ==========")

      result = self.send_goal_to_block_detection()
      rospy.loginfo("========== Find %d Cubes =========="%(len(result.block_pose.poses)))


      pick_pose_goal = PoseStamped()
      place_pose_goal = PoseStamped()
      pick_pose_goal.header = place_pose_goal.header = result.block_pose.header
      place_pose_goal.pose = self.place_pose_gen(0)
      goal_block_pick_and_place = block_pick_and_placeGoal(mode=goal.mode, object_size=self.gripper_size[goal.size_index])



      if goal.mode == 0:
        for i in range(len(result.block_pose.poses)):
          pick_pose_goal.pose = result.block_pose.poses[i]
          pick_pose = self.pose_transform(pick_pose_goal)
          if i == 0:
            place_pose_goal.pose.position.z = pick_pose.pose.position.z

          goal_block_pick_and_place.pick_pose.append(copy.deepcopy(pick_pose))
          goal_block_pick_and_place.place_pose.append(copy.deepcopy(place_pose_goal))
          place_pose_goal.pose.position.z += 0.02
         
      elif goal.mode == 1:
        if len(goal.color_order) == 0:
          rospy.loginfo("========== No color order, Task cancelled ==========")
          return

        for order_id in goal.color_order:
          # order_index = [i for i,x in enumerate(result.color_id) if x == order_id]
          # if len(order_index) > 0:
            # for o_index in order_index:
            #   pick_pose_goal.pose = result.block_pose.poses[order_index]
            #   pick_pose = self.pose_transform(pick_pose_goal)
            #   if not self.on_place_pose(pick_pose.pose, place_pose_goal.pose):
            #     break
          if (order_id in result.color_id):
            order_index = result.color_id.index(order_id)

            pick_pose_goal.pose = result.block_pose.poses[order_index]
            pick_pose = self.pose_transform(pick_pose_goal)  
            goal_block_pick_and_place.pick_pose.append(copy.deepcopy(pick_pose))
            goal_block_pick_and_place.place_pose.append(copy.deepcopy(place_pose_goal))

            place_pose_goal.pose.position.z += 0.0105
            result.block_pose.poses[order_index].position.z -= 0.01

          else:
            rospy.loginfo("========== Cannot Find color_id: %d =========="%(order_id))
            self._result.state = False
            self._as.set_aborted(self._result)

      self.client_2.send_goal(goal_block_pick_and_place)
      self.client_2.wait_for_result()
      result_2 = self.client_2.get_result()
      if not result_2.state:
        rospy.loginfo("========== Task Failed ==========")
        self._result.state = False
        self._as.set_aborted(self._result)


  def classifier_mode(self, goal):
      rospy.loginfo("========== Start Classifier Mode ==========")

      pick_pose_goal = PoseStamped()
      place_pose_goal = PoseStamped()
      goal_block_pick_and_place = block_pick_and_placeGoal(mode=goal.mode)
      place_pose_list = [self.place_pose_gen(0), self.place_pose_gen(1), self.place_pose_gen(3), self.place_pose_gen(4)]

      for i in range(goal.classifier_num):
        result = self.send_goal_to_block_detection()
        pick_pose_goal.header = place_pose_goal.header = result.block_pose.header
        cube_index = 0
        for block in result.block_pose.poses:
          pick_pose_goal.pose = block
          pick_pose = self.pose_transform(pick_pose_goal)
          if self.on_place_pose(pick_pose.pose,self.place_pose_gen(2)):
            goal_block_pick_and_place.pick_pose.append(pick_pose)
            break

          cube_index += 1


        place_pose_goal.pose = place_pose_list[result.color_id[cube_index]]
        goal_block_pick_and_place.place_pose.append(copy.deepcopy(place_pose_goal))
        place_pose_list[result.color_id[cube_index]].position.z += 0.0105

        self.client_2.send_goal(goal_block_pick_and_place)
        self.client_2.wait_for_result()
        goal_block_pick_and_place.pick_pose = []
        goal_block_pick_and_place.place_pose = []


  def on_place_pose(sefl, pick_pose, place_pose):
      x1 = pick_pose.position.x
      y1 = pick_pose.position.y
      x2 = place_pose.position.x
      y2 = place_pose.position.y
      delta_x = x1 - x2
      delta_y = y1 - y2

      dist = np.sqrt([delta_x**2 + delta_y**2])

      if dist > 0.01:
        return False
      else:
        return True

  def send_goal_to_block_detection(self):
      goal_block_detection = block_detectionGoal(start=True)
      self.client_1.send_goal(goal_block_detection)
      self.client_1.wait_for_result()
      return self.client_1.get_result()
  

  def onShutdown(self):
      rospy.loginfo("[%s] Shutting down..." %self.node_name)
      rospy.sleep(0.5) #To make sure that it gets published.
      rospy.loginfo("[%s] Shutdown" %self.node_name)


if __name__ == '__main__': 
    rospy.init_node('pick_and_place_server',anonymous=False)
    pick_and_place_server = pick_and_place_server()
    rospy.on_shutdown(pick_and_place_server.onShutdown)
    rospy.spin()
