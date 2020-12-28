#!/usr/bin/env python3.8

import actionlib
import os
import rospy
from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal

from interactions import DemoInteraction

from interaction_engine.json_database import Database
from interaction_engine.interfaces import Interface
from interaction_engine.messager import Message


class CordialInterface(Interface):

    def __init__(
            self,
            action_name="cordial/say_and_ask_on_gui",
            database_file_name=None,
            seconds_until_timeout=None,

            is_create_db_key_if_not_exist=True,
            node_name='cordial_client',
    ):
        database = Database(database_file_name)
        super().__init__(
            self.call_ask_action_service,
            database=database,
            is_create_db_key_if_not_exist=is_create_db_key_if_not_exist
        )

        self._action_name = action_name
        self._cordial_action_client = actionlib.SimpleActionClient(self._action_name, AskOnGuiAction)
        self._is_begin_interaction = False

        if seconds_until_timeout is not None:
            if type(seconds_until_timeout) is not int or not float:
                raise TypeError("Timeout must be int or float")
            if seconds_until_timeout <= 0:
                raise ValueError("Timeout must be greater than 0")
        self._seconds_until_timeout = seconds_until_timeout

        rospy.init_node(node_name)

        rospy.on_shutdown(self._cancel_goal)

    def call_ask_action_service(self, message):
        rospy.loginfo("Calling action")

        if type(message) is not Message:
            raise TypeError("Invalid message class.")

        goal = AskOnGuiGoal()
        goal.type = message.message_type
        goal.content = message.content
        goal.options = message.options
        goal.args = message.args

        self._cordial_action_client.wait_for_server()

        self._cordial_action_client.send_goal(goal, feedback_cb=self._cordial_feedback_cb)
        self._cordial_action_client.wait_for_result()
        response = self._cordial_action_client.get_result()

        return response.data

    def _cordial_feedback_cb(self, feedback):
        if self._seconds_until_timeout is not None:
            if feedback.time_passed > self._seconds_until_timeout:
                rospy.loginfo("Time passed: {}; goal cancelled".format(feedback.time_passed))
                self._cordial_action_client.cancel_goal()

    def _cancel_goal(self):
        if self._cordial_action_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
            self._cordial_action_client.cancel_goal()


if __name__ == "__main__":

    resources_directory = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources'
    demo_interaction_json_file = os.path.join(resources_directory, 'sar_demo_nodes.json')
    variation_file_name = os.path.join(resources_directory, 'variations.json')
    database_file_name = os.path.join(resources_directory, 'state_database.json')

    interface = CordialInterface(database_file_name=database_file_name)
    demo_interaction = DemoInteraction(
        interaction_json_file=demo_interaction_json_file,
        start_node_name="ask to chat",
        database_file_name=database_file_name,
        variation_file_name=variation_file_name,
        interface=interface
    )

    while not rospy.is_shutdown():
        demo_interaction.run_once()
        rospy.sleep(25)
