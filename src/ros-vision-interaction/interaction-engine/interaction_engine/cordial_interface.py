#!/usr/bin/env python
import actionlib
import logging
import os
import random
import rospy

from interaction_engine.database import Database
from interaction_engine.int_engine import InteractionEngine
from interaction_engine.interface import Interface
from interaction_engine.message import Message
from interaction_engine.state import State
from interaction_engine.state_collection import StateCollection

from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal, MouseEvent
from std_msgs.msg import Bool

logging.basicConfig(level=logging.INFO)


class CordialInterface(Interface):

    def __init__(self, action_name, seconds_until_timeout=None):

        super(CordialInterface, self).__init__(
            output_function=None,
            input_function=self.call_ask_action_service
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

        rospy.init_node('cordial_client')

        rospy.on_shutdown(self._cancel_goal)

    def _format_user_input(self, message, user_input):
        if message.message_type in [Message.Type.MULTIPLE_CHOICE, Message.Type.MULTIPLE_CHOICE_ONE_COLUMN]:
            user_input = message.options.index(user_input)
        return user_input

    def call_ask_action_service(self, message, data_dict):
        rospy.loginfo("Calling action")

        if type(message) is not Message:
            raise TypeError("Invalid message class.")

        updated_message_content = self.update_message(message, data_dict)

        goal = AskOnGuiGoal()
        goal.type = message.message_type
        goal.content = updated_message_content
        goal.options = message.options
        goal.args = message.args

        self._cordial_action_client.wait_for_server()

        self._cordial_action_client.send_goal(goal, feedback_cb=self._cordial_feedback_cb)
        self._cordial_action_client.wait_for_result()
        response = self._cordial_action_client.get_result()

        if response.data is "":
            response.data = message.default_input

        return self._format_user_input(message, response.data)

    def _cordial_feedback_cb(self, feedback):
        if self._seconds_until_timeout is not None:
            if feedback.time_passed > self._seconds_until_timeout:
                print("Time passed: {}; goal cancelled".format(feedback.time_passed))
                self._cordial_action_client.cancel_goal()

    def _cancel_goal(self):
        if self._cordial_action_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
            self._cordial_action_client.cancel_goal()
