#!/usr/bin/python3.8
import actionlib
import datetime
import os
import pymongo
import rospy
import schedule

from mongodb_statedb import StateDb
from ros_vision_interaction.msg import StartInteractionAction, StartInteractionFeedback, StartInteractionResult
from vision_interaction.controllers import InteractionManager
from vision_interaction.interfaces import CordialInterface

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class RosInteractionManager:

    def __init__(
            self,
            interaction_manager,
            start_interaction_action_name=START_INTERACTION_ACTION_NAME,
    ):
        self._interaction_manager = interaction_manager

        # set up action server
        self._start_interaction_action_server = actionlib.SimpleActionServer(
            start_interaction_action_name,
            StartInteractionAction,
            self.run_manager_once,
            auto_start=False
        )
        self._start_interaction_action_server.register_preempt_callback(self._preempt_callback)

        self._is_debug = rospy.get_param(
            "controllers/is_debug",
            False
        )

        self._update_scheduler = schedule.Scheduler()
        self._update_scheduler.every(15).seconds.do(self._update)

        self._start_interaction_action_server.start()

    def _update(self):
        pass

    # start interaction action callback
    def run_manager_once(self, goal):
        interaction_type = goal.type

        result = StartInteractionResult()

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

    def _preempt_callback(self):
        rospy.loginfo("Preempt requested for interaction server")
        self._start_interaction_action_server.set_preempted()


if __name__ == "__main__":
    rospy.init_node("interaction_manager")

    # set up state database
    host = rospy.get_param(
        "mongodb/host",
        "localhost"
    )
    port = rospy.get_param(
        "mongodb/port",
        62345
    )
    state_database = StateDb(
        pymongo.MongoClient(host, port)
    )

    interface = CordialInterface(state_database)

    interaction_manager = InteractionManager(
        mongodb_statedb=state_database,
        interface=interface
    )

    ros_interaction_manager = RosInteractionManager(interaction_manager)

    rospy.spin()
