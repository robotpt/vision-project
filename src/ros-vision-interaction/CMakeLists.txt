cmake_minimum_required(VERSION 2.8.3)
project(ros-vision-interaction)

find_package(catkin REQUIRED COMPONENTS
  cordial_msgs
  rospy
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
