cmake_minimum_required(VERSION 2.8.3)
project(ros_vision_interaction)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  cordial_msgs
  rospy
)

add_action_files(
  FILES
  StartInteraction.action
)

catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_behavior_controller_ros.test)
endif()

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
