## How to run
```
 $ roslaunch sis_mobile config.launch mecanum:=false
 $ roslaunch sis_mobile move_base.launch mecanum:=false
```
Set mecanum to true if you use mecanum mobile robot
## Usage example
After you launch two files above,
```
 $ rosrun sis_mobile move_base_client
```
After you run the node, the robot will start to traverse around a 0.6 meter square
#### If you meet any problem during using navigation module, please contact with TA_Sean

## Launch
* config.launch
  * arguments
    * port: arduino part path
    * mecanum: if use mecanum mobile robot, set to true
    * x: robot start pose x w.r.t. map frame
    * y: robot start pose y w.r.t. map frame
    * th: robot start pose yaw w.r.t. map frame
  * sublaunchs
    * static_transform.launch
      * arguments
        * mecanum
        * x
        * y 
        * th
    * controller.launch
      * arguments
        * port
        * mecanum
    * localization.launch
    * camera.launch
    * apriltags.launch

## Nodes
* **pid_controller**
  * Publish topic
    * [/wheel_odom](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)
  * Subscribe topic
    * [/cmd_vel](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
  * Service
    * [/reset_wheel_odom](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)
* **pose_from_tags**
  * Publish topic
    * [/car_pose_from_tag](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
  * Subscribe topic
    * [/tag_detections](http://docs.ros.org/kinetic/api/apriltags2_ros/html/msg/AprilTagDetectionArray.html)
* **sw_pid_controller**
  * Publish topic
    * [/wheel_odom](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)
  * Subscribe topic
    * [/cmd_vel](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
  * Service
    * [/reset_wheel_odom](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)
    
## Parameters
* **move_base related**
  * **base_local_planner_params.yaml**
    * max_vel_x: maximum forward velocity allowed for the base in meters/sec
    * min_vel_x: minimum forward velocity allowed for the base in meters/sec
    * max_vel_theta: maximum rotational velocity allowed for the base in radians/sec
    * min_vel_theta: minimum rotational velocity allowed for the base in radians/sec
    * min_in_place_vel_theta: minimum rotational velocity allowed for the base while performing in-place rotations in radians/sec
    * acc_lim_theta: rotational acceleration limit of the robot in radians/sec^2
    * acc_lim_x: x acceleration limit of the robot in meters/sec^2
    * acc_lim_y: y acceleration limit of the robot in meters/sec^2
    * xy_goal_tolerance: tolerance in meters for the controller in the x & y distance when achieving a goal
    * yaw_goal_tolerance: tolerance in radians for the controller in yaw/rotation when achieving its goal
    * latch_xy_goal_tolerance: if goal tolerance is latched, if the robot ever reaches the goal xy location it will simply rotate in place, even if it ends up outside the goal tolerance while it is doing so
    * holonomic_robot: determines whether velocity commands are generated for a holonomic or non-holonomic robot. For holonomic robots, strafing velocity commands may be issued to the base. For non-holonomic robots, no strafing velocity commands will be issued
  * **costmap_common_params.yaml**
    * obstacle_range: maximum range sensor reading that will result in an obstacle being put into the costmap
    * raytrace_range: range to which we will raytrace freespace given a sensor reading
    * footprint: footprint of the robot
    * inflation_radius: set to the maximum distance from obstacles at which a cost should be incurred
  * **global_costmap_params.yaml**
    * global_frame: global frame for the costmap to operate in
    * robot_base_frame: name of the frame for the base link of the robot
    * update_frequency: frequency in Hz for the map to be updated
    * static_map: whether or not the costmap should initialize itself based on a map served by the map_server. 
    * rolling_window: whether or not to use a rolling window version of the costmap. If the static_map parameter is set to true, this parameter must be set to false.
    * width: width of the map in meters
    * height: height of the map in meters
    * resolution: resolution of the map in meters/cell
  * **local_costmap_params.yaml**
    * publish_frequency: frequency in Hz for the map to be publish display information
    ###### Other parameters are same as above one
  * **sw_base_local_planner_params.yaml**
    * acc_lim_x: x acceleration limit of the robot in meters/sec^2
    * acc_lim_y: y acceleration limit of the robot in meters/sec^2
    * acc_lim_th: rotational acceleration limit of the robot in radians/sec^2
    * max_vel_x: maximum x velocity for the robot in m/s
    * min_vel_x: minimum x velocity for the robot in m/s, negative for backwards motion
    * max_vel_y: maximum y velocity for the robot in m/s
    * min_vel_y: minimum y velocity for the robot in m/s
    * max_trans_vel: absolute value of the maximum translational velocity for the robot in m/s
    * min_trans_vel: absolute value of the minimum translational velocity for the robot in m/s
    * max_rot_vel: absolute value of the maximum rotational velocity for the robot in rad/s 
    * min_rot_vel: absolute value of the minimum rotational velocity for the robot in rad/s
    * yaw_goal_tolerance: tolerance in radians for the controller in yaw/rotation when achieving its goal 
    * xy_goal_tolerance: tolerance in meters for the controller in the x & y distance when achieving a goal
    * latch_xy_goal_tolerance: if goal tolerance is latched, if the robot ever reaches the goal xy location it will simply rotate in place, even if it ends up outside the goal tolerance while it is doing so
    ###### This file is similiar with base_local_planner_params.yaml while this is for mecanum mobile robot
* **map related**
  * **sis_competition_map.yaml**
    * image: path to the image file containing the occupancy data
    * resolution: resolution of the map, meters / pixel
    * origin: the 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
    * occupied_thresh: pixels with occupancy probability greater than this threshold are considered completely occupied
    * free_thresh: pixels with occupancy probability less than this threshold are considered completely free
    * negate: whether the white/black free/occupied semantics should be reversed
* **localization related**
  * **robot_ekf_config.yaml**
    * frequency: the filter produces a state estimate
    * two_d_mode: if the robot operate in a planar environment
    * publish_tf: if the node will publish the transform from **world_frame** to **base_link_frame**
    * map_frame: a world-fixed frame, while it contains the most globally accurate position estimate for the robot, it is subject to discrete jumps
    * odom_frame: the robot position in this frame will drift over time, but is accurate in the short term and sholbe be continuous
    * base_link_frame: frame that is affixed to the robot
    * world_frame: default to odom_frame
    * odom0: pose and twist from wheel odometry
    * odom0_config: what variables of odom0 should be fused into the final state estimate 
    * odom0_differential: whether the psoe variables should be integrated differentially
    * odom0_relative: if true, then any measurements from this sensor will be fused relative to the first measurement received fromm that sensor
    * odom0_pose_rejection_threshold: if data is subject to outliers, use this threshold setting expressed as Mahalanobis distances to control how far away from the current vehicle state a sensor measurement is permitted to
    * odom0_twist_rejection_threshold: similiar as above
    * pose0: pose estimation from apriltag(s)
    * use_control: if truem the node will subscribe to *cmd_vel* topic and use that to generate an acceleration term. This term then used in the robot's state prediction

