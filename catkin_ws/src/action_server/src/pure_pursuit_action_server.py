#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from sis_arm_msgs.msg import Odom_uno
from action_server.msg import Astar_followingAction, Astar_followingGoal, path_followingAction, path_followingFeedback, path_followingResult, Point2D, path_followingArrayAction, path_followingArrayFeedback, path_followingArrayResult
import actionlib
import numpy as np
import math
from dagu_car.msg import  Twist2DStamped

"""
This program utilizes pure pursuit to follow a given trajectory.
"""

class path_following_action():
    def __init__(self):

        # Init attributes
        self.threshold_proximity = rospy.get_param('~threshold_proximity',0.1)      # How close the robot needs to be to the final waypoint to stop driving
        
        self.waypoints = []
        self.speed = 0.4
        self.r_speed = 0.2
        self.state = False
        self.switch = False
        self.err_ = 0
        self.rotation_switch = False
        self.astar_switch = False
        self.target_msg = Point2D()

        self.distance_from_path = None
        self.destination_pose = None
        self.tag_count = 0

        self.robot_pose = (0, 0, 0)#(-0.3, -0.1789, -0.0246)


        self.action_state = rospy.get_param("~action_state", 0)
        # Init subscribers and publishers
        if self.action_state == 1:
            self.sub_odom = rospy.Subscriber("/odom", Odom_uno, self.cbOdometry, queue_size=1)
            self.astar_timer = rospy.Timer(rospy.Duration(1), self.a_star_timer)
            self.client = actionlib.SimpleActionClient('octomap_path_finding_server', Astar_followingAction)
            self.client.wait_for_server()
            self.PF_action = actionlib.SimpleActionServer("path_following_action", path_followingAction, execute_cb=None, auto_start = False)
            self.PF_action.register_goal_callback(self.actionCB)
            self.path_following_feedback = path_followingFeedback()
            self.path_following_result = path_followingResult()
        else:
            self.PF_action = actionlib.SimpleActionServer("path_following_action", path_followingArrayAction, execute_cb=None, auto_start = False)
            self.PF_action.register_goal_callback(self.actionArrayCB)
            self.path_following_feedback = path_followingArrayFeedback()
            self.path_following_result = path_followingArrayResult()

        self.pub_car_cmd = rospy.Publisher("/car_cmd", Twist2DStamped, queue_size=1)
        self.sub_change_speed = rospy.Subscriber("/car_linear_speed", Float32, self.cbLinearSpeed, queue_size=1)
        self.sub_change_speed = rospy.Subscriber("/car_angular_speed", Float32, self.cbAngularSpeed, queue_size=1)

        self.PF_action.start()

        rospy.loginfo('Path_following_action: Initialized')


        self.init_param()

    def cbLinearSpeed(self, msg):
        self.speed = msg.data

    def cbAngularSpeed(self, msg):
        self.r_speed = msg.data

    def a_star_timer(self, event):
        if self.astar_switch:
          self.a_star_action()

    def a_star_action(self):
        start_msg = Point2D()
        start_msg.x = self.robot_pose[0]
        start_msg.y = self.robot_pose[1]

        goal_path_finding = Astar_followingGoal(target_point=self.target_msg, start_point=start_msg)
        self.client.send_goal(goal_path_finding)
        self.client.wait_for_result()
        result = self.client.get_result()
        # print result
        # print len(result.waypoints)
        if len(result.waypoints) == 0:
          return

        self.waypoints_new = []
        for waypoint in result.waypoints:
          self.waypoints_new.append((waypoint.x, waypoint.y))

        rospy.loginfo('Finding New Path')
        self.waypoints = self.waypoints_new
        self.init_param()

    # def cbApriltags(self, tag_msg):

    #     # print tag_msg
    #     if self.go_to_tag_flag == 1:
    #       self.turn_around(0)

    #     elif self.go_to_tag_flag == 2:
    #       if len(tag_msg.detections) == 0:
    #         return
    #       tag_x = tag_msg.detections[0].pose.pose.pose.position.z
    #       tag_y = - tag_msg.detections[0].pose.pose.pose.position.x
    #       print "tag_x, tag_y = ", round(tag_x, 3)," ", round(tag_y, 3)
    #       if tag_x > 0.41:
    #         self.car_cmd_pub(0.05, tag_y * 5)
    #       else:
    #         self.car_cmd_pub(0, 0)
    #         self.go_to_tag_flag = 3

    #     elif self.go_to_tag_flag == 3:
    #         raw_input("PAUSE")
    #         self.turn_around(1)

    def actionCB(self):
        goal = self.PF_action.accept_new_goal()
        self.target_msg.x = goal.target_point.x
        self.target_msg.y = goal.target_point.y
        self.astar_switch = True
        self.waypoints.append((self.target_msg.x, self.target_msg.y))
        self.a_star_action()

    def actionArrayCB(self):
        goal = self.PF_action.accept_new_goal()
        if goal.mode == 0:
            for waypoint in goal.waypoints:
              self.waypoints.append((waypoint.x, waypoint.y))
            rospy.loginfo('Follow New Path')
            # print self.waypoints
            self.init_param()

        else:
            rospy.loginfo('Execute Car Action')
            self.car_action(goal)
            self.action_succeeded()


    def init_param(self):
        self.current_waypoint_index = 0
        self.lookahead_distance = 0.01
        self.switch = False


    def action_succeeded(self):
        self.car_cmd_pub(0,0)
        self.err_ = 0
        self.state = False
        self.switch = False
        self.astar_switch = False
        self.path_following_result.state = True
        self.waypoints = []
        rospy.loginfo('Path_Following: Succeeded')
        self.PF_action.set_succeeded(self.path_following_result)

    def car_cmd_pub(self, v, w):
        car_cmd = Twist2DStamped()
        car_cmd.v = v
        car_cmd.omega = w

        self.pub_car_cmd.publish(car_cmd)

    def cbOdometry(self, msg):
        if self.action_state == 1:
            self.robot_pose = (msg.odom.x, msg.odom.y, msg.odom.theta)
        else:
            self.robot_pose = (msg.x, msg.y, msg.z)


        if not self.PF_action.is_active():
            return 

        self.start_pure_pursuit()
        self.path_following_feedback.dist = self.getDistance(self.robot_pose, self.waypoints[-1])
        self.PF_action.publish_feedback(self.path_following_feedback)

    def last_point(self, yaw):

        #dist = math.hypot((self.robot_pose[0] - self.waypoints[-1][0]), (self.robot_pose[1] - self.waypoints[-1][1]))
        dist = self.getDistance(self.robot_pose, self.waypoints[-1])

        print "DIST_last_point: ",round(dist, 3),"\n"
        if dist < 0.01:
          self.action_succeeded()

        else:
          err = yaw
          if abs(err) <0.08:
            self.car_cmd_pub(0.05, 0)

          else:
            self.err_ += err 
            self.car_cmd_pub(0, self.err_)

        if self.err_ > 2:
              self.err_ = 2
        if self.err_ < -2:
              self.err_ = -2

    def start_pure_pursuit(self):
        if self.switch:
            self.last_point(self.getAngle(self.robot_pose, self.waypoints[-1]))
            return

        try:
          self.destination_pose = self.circleIntersect(self.lookahead_distance)

        except Exception as e:
          print "------------------ Run Time error ---------------------"
          rospy.loginfo(e)
          # self.switch = True
          return
          #self.last_point(self.getAngle(self.robot_pose, self.waypoints[-1]))
          #raw_input("press enter to continue")

        if self.destination_pose == None:
          self.switch = True
          return
        
        dist = self.getDistance(self.robot_pose, self.waypoints[-1])

        if dist > 0.05:
          # print "DIST: ",round(dist, 3)
          angle_ratio = 20
          self.execute_car_cmd(angle_ratio)


        else:
          self.switch = True


    def execute_car_cmd(self, angle_ratio):
        # distance_to_destination= self.getDistance(self.robot_pose, self.destination_pose)
        angle_to_destination = self.getAngle(self.robot_pose, self.destination_pose) 
        #print "angle_to_destination", round(angle_to_destination * 180 / math.pi, 2), "\n"      
        w = angle_to_destination * angle_ratio
        if (abs(angle_to_destination) > 1.57) or (self.rotation_switch == True):
          self.rotation_switch = True
          if w > 0:
            self.car_cmd_pub(0, self.r_speed)
          else:
            self.car_cmd_pub(0, -self.r_speed)

          if abs(angle_to_destination) < 0.26:
              self.rotation_switch = False
        else:
            self.car_cmd_pub(self.speed, w)

    def car_action(self, goal):
        if goal.mode == 1:
            while( abs(angle) > 0.1):
              if angle > 0:
                self.car_cmd_pub(0, -self.r_speed)
              else:
                self.car_cmd_pub(0, self.r_speed)

              angle = self.getAngle(self.robot_pose, (goal.tag_point.x, goal.tag_point.y))

        elif goal.mode == 2:
            self.car_cmd_pub(-self.speed,0)
            rospy.sleep(2)
            self.car_cmd_pub(0, 0)


    def distanceBtwnPoints(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def isPointOnLineSegment(self, x, y, x_start, y_start, x_end, y_end):
        return round(self.distanceBtwnPoints(x_start, y_start, x, y) + self.distanceBtwnPoints(x, y, x_end, y_end), 5) == round(self.distanceBtwnPoints(x_start, y_start, x_end, y_end), 5)

    # Find point on path that is closest to current robot position
    def closestPoint(self):
        x_robot, y_robot = self.robot_pose[:2]

        # Initialize values
        x_closest, y_closest = None, None
        waypoint_index = self.current_waypoint_index
        shortest_distance = self.distanceBtwnPoints(x_robot, y_robot, self.waypoints[waypoint_index][0], self.waypoints[waypoint_index][1]) #float('inf')

        for i in range(self.current_waypoint_index, len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i+1]

            # For line (segment) equation ax + by + c = 0
            a = y1 - y2
            b = x2 - x1
            c = -b*y1 - a*x1  # Equivalently: x1*y2 - x2*y1
            # print x1,y1
            # print x2,y2,"\n"
            x_close = (b*(b*x_robot - a*y_robot) - a*c)/(a**2 + b**2)
            y_close = (a*(-b*x_robot + a*y_robot) - b*c)/(a**2 + b**2)

            if not self.isPointOnLineSegment(x_close, y_close, x1, y1, x2, y2):
                continue

            distance = self.distanceBtwnPoints(x_robot, y_robot, x_close, y_close)

            if distance < shortest_distance:
                shortest_distance = distance
                x_closest = x_close
                y_closest = y_close
                waypoint_index = i

        # if waypoint_index != self.current_waypoint_index:
        #     # print "current waypoint_index is No.%d"%(waypoint_index)

        self.current_waypoint_index = waypoint_index
        self.distance_from_path = shortest_distance
        return (x_closest, y_closest)

    # Find next point along the path at which the lookahead distance "circle" intersects, or None if we can stop driving
    def circleIntersect(self, lookahead_distance):
        if len(self.waypoints) < 2:
            return None

        # If we are at the second-to-last waypoint and the robot is close enough to final waypoint, return None (to signal "stop")
        if self.current_waypoint_index == len(self.waypoints) - 2:
            x_endpoint, y_endpoint = self.waypoints[-1]
            x_robot, y_robot = self.robot_pose[:2]
            if self.distanceBtwnPoints(x_endpoint, y_endpoint, x_robot, y_robot) <= self.threshold_proximity:
                return None
            else:
                return self.waypoints[-1]

    # We only want to search the first path line segment past the point at which the robot is, so we make a "fake waypoint" for the current location of the robot along the path, and search from there onwards
        fake_robot_waypoint = self.closestPoint()
        if fake_robot_waypoint == (None, None):
            return self.waypoints[self.current_waypoint_index + 1]
        
        waypoints_to_search = [fake_robot_waypoint] + self.waypoints[self.current_waypoint_index + 1 : ]

        # If lookahead distance is shorter than distance from path, recall function with larger lookahead distance
        if lookahead_distance < self.distance_from_path:
            return self.circleIntersect(self.distance_from_path + 0.05)

        # For circle equation (x - p)^2 + (y - q)^2 = r^2
        p, q = self.robot_pose[:2]
        r = lookahead_distance

        # Check line segments along path until intersection point is found or we run out of waypoints
        for i in range(len(waypoints_to_search) - 1):
            # For line (segment) equation y = mx + b
            x1, y1 = waypoints_to_search[i]
            x2, y2 = waypoints_to_search[i+1]

            if x2 - x1 != 0:
                m = (y2 - y1)/(x2 - x1)
                b = y1 - m*x1

                #print "r=", r

                # Quadratic equation to solve for x-coordinate of intersection point
                A = m**2 + 1
                B = 2*(m*b - m*q - p)
                C = q**2 - r**2 + p**2 - 2*b*q + b**2

                if B**2 - 4*A*C < 0:    # Circle does not intersect line
                    continue
                
                # Points of intersection (could be the same if circle is tangent to line)
                x_intersect1 = (-B + math.sqrt(B**2 - 4*A*C))/(2*A)
                x_intersect2 = (-B - math.sqrt(B**2 - 4*A*C))/(2*A)
                y_intersect1 = m*x_intersect1 + b
                y_intersect2 = m*x_intersect2 + b
            else:
                x_intersect1 = x1
                x_intersect2 = x1
                y_intersect1 = q - math.sqrt(-x1**2 + 2*x1*p - p**2 + r**2)
                y_intersect2 = q + math.sqrt(-x1**2 + 2*x1*p - p**2 + r**2)

            # See if intersection points are on this specific segment of the line
            #print 'x_int, yint', x_intersect1, y_intersect1, x_intersect2, y_intersect2
            #print 'waypoints', x1, y1, x2, y2
            if self.isPointOnLineSegment(x_intersect1, y_intersect1, x1, y1, x2, y2):
                #rospy.loginfo('is returning')
                return (x_intersect1, y_intersect1)
            elif self.isPointOnLineSegment(x_intersect2, y_intersect2, x1, y1, x2, y2):
                #rospy.loginfo('is returning2')
                return (x_intersect2, y_intersect2)

        # If lookahead circle does not intersect the path at all (and the other two conditions at beginning of this function failed), then reduce the lookahead distance
        return self.circleIntersect(lookahead_distance - 0.05)

    def getDistance(self, start_pose, end_pose):
        #Takes a starting coordinate (x,y,theta) and ending coordinate (x,y) and returns distance between them in map units
        delta_x = end_pose[0] - start_pose[0]
        delta_y = end_pose[1] - start_pose[1]

        distance = np.sqrt([delta_x**2 + delta_y**2])
        return distance[0]

    def getAngle(self, start_pose, end_pose):
        #Takes a starting coordinate (x,y,theta) and ending coordinate (x,y) and returns angle between them relative to the front of the car in degrees
        delta_x = end_pose[0] - start_pose[0]
        delta_y = end_pose[1] - start_pose[1]

        #rad_to_deg_conv = 180.0 / np.pi
        theta = start_pose[2] #* rad_to_deg_conv
     
        between_angle = np.arctan2(delta_y, delta_x) #* rad_to_deg_conv

        #print "theta", theta*180/math.pi
        #print "between_angle", between_angle*180/math.pi

        return self.correctAngle(theta, between_angle)

    def correctAngle(self, robot_angle, destination_angle):
        angle = abs(robot_angle - destination_angle)
        if math.pi/2 < robot_angle and robot_angle < math.pi and angle > math.pi:
            return destination_angle - robot_angle + math.pi*2  

        elif -(math.pi) < robot_angle and robot_angle < -(math.pi/2) and angle > math.pi:
            return destination_angle - robot_angle - math.pi*2

        else:
            return destination_angle - robot_angle 


if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("path_following_action",anonymous=False)
    path_following_action = path_following_action()
    rospy.spin()
