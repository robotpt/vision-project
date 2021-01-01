#!/usr/bin/env python

import actionlib
import mock
import rospy
import rostest
import unittest

from controllers.behavior_controller import BehaviorController
from ros_vision_interaction.msg import StartInteractionAction, StartInteractionGoal, StartInteractionFeedback

PKG = "ros_vision_interaction"
NODE_NAME = "test_behavior_controller"
TEST_CONTROLLER_NAME = "test_behavior_controller"
TEST_ACTIONS_NAME = "test_behavior_controller_actions"
_START_INTERACTION_ACTION = "vision_project/start_interaction"


class TestBehaviorController(unittest.TestCase):

    @mock.patch('builtins.open')
    @mock.patch('controllers.behavior_controller.Database._load_database_from_file')
    def setUp(self, mock_load_db_from_file, mock_open):
        mock_load_db_from_file.return_value = {"user_name": ""}
        mock_open.side_effect = mock.mock_open(read_data="{}")
        self._behavior_controller = BehaviorController(
            "interactions.json",
            "state_db.json"
        )

    def test_build_graph_from_json(self):
        test_graph_dict = {
            "graph_name": "test_directed_graph",
            "start_node_name": "node1",
            "nodes": {
                "node1": {
                    "transitions": ["node1", "node2"],
                    "content": "node1 content",
                    "options": ["option1-1", "option1-2"],
                    "message_type": "multiple choice"
                },
                "node2": {
                    "transitions": ["node1", "exit"],
                    "content": "node2 content",
                    "options": ["option2-1", "option2-exit"],
                    "message_type": "multiple choice"
                },
            }
        }

        test_directed_graph = self._behavior_controller._build_graph_from_json(test_graph_dict)
        nodes_dict_from_graph = test_directed_graph._nodes_dict

        self.assertIn("node1", nodes_dict_from_graph.keys())
        self.assertIn("node2", nodes_dict_from_graph.keys())

        expected_nodes_dict = test_graph_dict["nodes"]
        for node in expected_nodes_dict:
            expected_transitions = expected_nodes_dict[node]["transitions"]
            actual_transitions = nodes_dict_from_graph[node].transitions
            self.assertEqual(
                expected_transitions,
                actual_transitions
            )

            expected_content = expected_nodes_dict[node]["content"]
            actual_content = nodes_dict_from_graph[node].message.content
            self.assertEqual(
                expected_content,
                actual_content
            )


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
    rospy.init_node(NODE_NAME)
    rostest.rosrun(PKG, TEST_CONTROLLER_NAME, TestBehaviorController)
    rostest.rosrun(PKG, TEST_ACTIONS_NAME, TestBehaviorControllerActions)
