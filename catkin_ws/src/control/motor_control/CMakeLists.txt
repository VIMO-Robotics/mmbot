cmake_minimum_required(VERSION 2.8.3)
project(motor_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

find_package(serial REQUIRED)

add_message_files(
  FILES
  motor_pwm.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS rt pthread
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(pid SHARED src/pid.cpp)

add_executable(receive_encoder src/receive_encoder.cpp)
target_link_libraries(receive_encoder ${serial_LIBRARIES} ${catkin_LIBRARIES} pid)

