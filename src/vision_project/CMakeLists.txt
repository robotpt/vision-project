cmake_minimum_required(VERSION 3.0.2)
project(vision_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
