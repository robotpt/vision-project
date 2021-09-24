#!/usr/bin/env python3.8

import actionlib
import rospy
from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal

from interaction_engine.interfaces import Interface
from interaction_engine.messager import Message


ERROR_RESPONSE = "ERROR"


class CordialInterface(Interface):

    def __init__(
            self,
            state_database,
            action_name="cordial/say_and_ask_on_gui",
            seconds_until_timeout=None,
            is_create_db_key_if_not_exist=True,
            default_response_index=0
    ):
        self._state_database = state_database
        super().__init__(
            self.call_ask_action_service,
            database=self._state_database,
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
        self._default_response_index = default_response_index

        rospy.on_shutdown(self.cancel_goal)

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

        if self._cordial_action_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            response = ERROR_RESPONSE

        return response.data

    def _cordial_feedback_cb(self, feedback):
        if self._seconds_until_timeout is not None:
            if feedback.time_passed > self._seconds_until_timeout:
                rospy.loginfo("Time passed: {}; goal cancelled".format(feedback.time_passed))
                self._cordial_action_client.cancel_goal()

    def cancel_goal(self):
        rospy.loginfo("Cordial interface canceling goal")
        if self._cordial_action_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
            self._cordial_action_client.cancel_goal()
