#!/usr/bin/env python3.8
import actionlib
import datetime
import os
import pprint
import pymongo
import rospy
import schedule

from vision_interaction.controllers import VisionProjectDelegator
from cordial_msgs.msg import MouseEvent
from mongodb_statedb import StateDb
from ros_vision_interaction.msg import StartInteractionAction, StartInteractionGoal
from std_msgs.msg import Bool

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class RosVisionProjectManager:

    def __init__(
            self,
            vision_project_delegator,
            is_go_to_sleep_topic='cordial/sleep',
            is_record_topic='data_capture/is_record',
            screen_tap_topic='cordial/gui/event/mouse',
            start_interaction_action_name=START_INTERACTION_ACTION_NAME,
    ):
        self._delegator = vision_project_delegator

        # action client to start interaction
        self._start_interaction_client = actionlib.SimpleActionClient(
            start_interaction_action_name,
            StartInteractionAction
        )

        # ROS publishers and subscribers
        self._is_record_publisher = rospy.Publisher(is_record_topic, Bool, queue_size=1)
        self._screen_tap_listener = rospy.Subscriber(
            screen_tap_topic,
            MouseEvent,
            callback=self._screen_tap_listener_callback,
            queue_size=1
        )
        self._sleep_publisher = rospy.Publisher(is_go_to_sleep_topic, Bool, queue_size=1)

        # update scheduler
        self._update_scheduler = schedule.Scheduler()
        self._update_scheduler.every(15).seconds.do(self.update)

        self._is_debug = rospy.get_param(
            "controllers/is_debug",
            False
        )

    def update(self):
        rospy.loginfo("Updating")

    def delegate_interaction(self):
        self._start_interaction_client.wait_for_server()

        interaction_type = self._delegator.determine_interaction_type()
        start_interaction_goal = StartInteractionGoal()
        start_interaction_goal.type = interaction_type

        if not self._is_debug:
            # TODO: add a feedback callback
            rospy.loginfo("Sending goal to start interaction")
            self._start_interaction_client.send_goal(start_interaction_goal)
            self._start_interaction_client.wait_for_result()

    def _get_time_string_without_milliseconds(self, time):
        return time.strftime("%H:%M:%S")

    def _screen_tap_listener_callback(self, _):
        self._is_start_interaction = True


if __name__ == "__main__":
    rospy.init_node("main_controller")

    resources_directory = rospy.get_param("vision-project/resources/path/deployment")

    # set up database
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

    vision_project_delegator = VisionProjectDelegator(
        mongodb_statedb=state_database,
        is_clear_state=True
    )

    ros_vision_project_manager = RosVisionProjectManager(vision_project_delegator)

    rospy.spin()
