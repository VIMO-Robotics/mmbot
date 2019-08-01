# Robot Navigation
This package contains one .srv file for the rosservice, two .py files, 
parking_service.py defines the service and demo.py demonstrate how to call the service in python.

# How to run
To run the demonstration, please put the robot in the middle of tag 4 and tag 6 and run with folowing command

this launch the tag_detection module and also serve as roscore
`
docker $ roslaunch sis_mobile tag_detection.launch
`

the x,y,th means the start postition of the robot
`
mm_pi $ roslaunch sis_mobile navigation_pi.launch x:=1 y:=0.9 th:=0 car_id:=[id ex:16 for tx2-16] (for diff. robot)
mm_pi $ roslaunch sis_mobile navigation_pi.launch x:=1 y:=0.9 th:=0 car_id:=[id ex:16 for tx2-16] mecanum:=true (for mecanum robot)
`

this launch the navigation service server
`
docker $ roslaunch robot_navigation robot_navigation.launch
`

`
docker $ rosrun robot_navigation demo.py
`

The robot will start to move to the front of apriltags with id 1 with distance 40 cm if it sees tag 1.
