#!/usr/bin/env python

import actionlib
import rospy
import rostest
import unittest

from ros_vision_interaction.msg import StartInteractionAction, StartInteractionGoal, StartInteractionFeedback

PKG = "ros_vision_interaction"
NAME = "test_behavior_controller"
_START_INTERACTION_ACTION = "vision_project/start_interaction"


class TestBehaviorControllerActions(unittest.TestCase):

    def test_start_interaction_action(self):
        client = actionlib.SimpleActionClient(
            _START_INTERACTION_ACTION,
            StartInteractionAction
        )

        goal = StartInteractionGoal()
        goal.type = "first interaction"

        self.assertTrue(
            client.wait_for_server(rospy.Duration(2))
            , "Could not connect"
        )
        rospy.loginfo("Connected to server")

        client.send_goal(goal)

        self.assertTrue(
            client.wait_for_result(rospy.Duration(5)),
            "Goal didn't finish"
        )
        self.assertEqual(actionlib.GoalStatus.SUCCEEDED, client.get_state())
        self.assertTrue(client.get_result().is_interaction_successful)


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestBehaviorControllerActions)
