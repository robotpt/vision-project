#!/usr/bin/env python

import actionlib
import rospy
import rostest
import unittest

from ros_vision_interaction.msg import StartInteractionAction, StartInteractionGoal

PKG = "ros_vision_interaction"
NAME = "test_main_controller"
_START_INTERACTION_ACTION = "vision_project/start_interaction"


class TestMainController(unittest.TestCase):

    pass


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestMainController)
