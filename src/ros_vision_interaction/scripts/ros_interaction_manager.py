#!/usr/bin/python3.8
import actionlib
import datetime
import os
import pymongo
import rospy
import schedule

from controllers import InteractionManager
from data_structures import param_database, state_database
from interfaces import CordialInterface

from cordial_msgs.msg import MouseEvent
from ros_vision_interaction.msg import StartInteractionAction, \
    StartInteractionGoal, \
    StartInteractionFeedback, \
    StartInteractionResult
from std_msgs.msg import Bool

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class RosInteractionManager:

    def __init__(
            self,
            interaction_manager,
            is_go_to_sleep_topic='cordial/sleep',
            screen_tap_topic='cordial/gui/event/mouse',
            start_interaction_action_name=START_INTERACTION_ACTION_NAME,
    ):
        self._interaction_manager = interaction_manager
        self._state_database = state_database
        self._is_engine_running = False

        self._screen_tap_listener = rospy.Subscriber(
            screen_tap_topic,
            MouseEvent,
            callback=self._screen_tap_listener_callback,
            queue_size=1
        )
        self._sleep_publisher = rospy.Publisher(is_go_to_sleep_topic, Bool, queue_size=1)

        # set up action server
        self._start_interaction_action_server = actionlib.SimpleActionServer(
            start_interaction_action_name,
            StartInteractionAction,
            self.run_manager_once,
            auto_start=False
        )
        self._start_interaction_action_server.register_preempt_callback(self._preempt_callback)

        self._is_run_demo = rospy.get_param(
            "vision-project/is_run_demo_interaction",
            False
        )

        self._is_debug = rospy.get_param(
            "vision-project/controllers/is_debug",
            False
        )

        self._start_interaction_action_server.start()

    # start interaction action callback
    def run_manager_once(self, goal):
        interaction_type = goal.type
        result = StartInteractionResult()
        self._is_engine_running = True

        if not self._is_debug:
            self._interaction_manager.run_interaction_once(interaction_type)
            rospy.loginfo("Interaction finished")
        else:
            seconds_to_sleep_for_tests = 3
            rospy.sleep(seconds_to_sleep_for_tests)
            result.is_interaction_successful = True

        if not self._start_interaction_action_server.is_preempt_requested():
            rospy.loginfo("Setting goal as succeeded")
            self._start_interaction_action_server.set_succeeded(result)

        self._state_database.set("last interaction time", datetime.datetime.now())
        self._is_engine_running = False

    def _preempt_callback(self):
        rospy.loginfo("Preempt requested for interaction server")
        self._start_interaction_action_server.set_preempted()

    def _screen_tap_listener_callback(self, _):
        if not self._is_engine_running and not self._is_run_demo:
            self.run_manager_once(StartInteractionGoal("prompted interaction"))

    @property
    def is_engine_running(self):
        return self._is_engine_running


if __name__ == "__main__":

    rospy.init_node("interaction_manager")
    seconds_until_timeout = rospy.get_param("vision-project/seconds_until_interaction_timeout")

    interface = CordialInterface(
        state_database,
        seconds_until_timeout=seconds_until_timeout
    )

    interaction_manager = InteractionManager(
        statedb=state_database,
        paramdb=param_database,
        interface=interface
    )

    ros_interaction_manager = RosInteractionManager(interaction_manager)

    rospy.spin()
