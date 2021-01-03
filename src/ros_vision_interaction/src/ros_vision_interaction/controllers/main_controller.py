#!/usr/bin/env python3.8
import actionlib
import datetime
import logging
import os
import pymongo
import rospy
import schedule

from mongodb_statedb import StateDb

from cordial_msgs.msg import MouseEvent
from mongodb_statedb import StateDb
from ros_vision_interaction.msg import StartInteractionAction, StartInteractionGoal
from std_msgs.msg import Bool

logging.basicConfig(level=logging.INFO)

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class MainController:

    def __init__(
        self,
        mongodb_statedb,
        state_database_file=None,
        is_go_to_sleep_topic='cordial/sleep',
        is_record_topic='data_capture/is_record',
        screen_tap_topic='cordial/gui/event/mouse',
        start_interaction_action_name=START_INTERACTION_ACTION_NAME,
        is_debug=False
    ):
        self._state_database = mongodb_statedb
        self._set_initial_db_keys()

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
        self._is_start_interaction = False

        self._update_scheduler = schedule.Scheduler()
        self._update_scheduler.every(15).seconds.do(self._update)

        self._is_debug = is_debug

    def _update(self):
        pass

    def _set_initial_db_keys(self):
        keys = [
            "user_name",
            "time_for_next_interaction"
        ]
        for key in keys:
            try:
                state_database.create(key, None)
            except KeyError:
                rospy.loginfo("{} already exists".format(key))

    def _screen_tap_listener_callback(self, _):
        self._is_start_interaction = True

    def delegate_interaction(self):
        self._start_interaction_client.wait_for_server()

        # determine interaction type from scheduler/state db
        interaction_type = None
        start_interaction_goal = StartInteractionGoal()
        start_interaction_goal.type = interaction_type

        if not self._is_debug:
            # TODO: add a feedback callback
            rospy.loginfo("Sending goal to start interaction")
            self._start_interaction_client.send_goal(start_interaction_goal)
            self._start_interaction_client.wait_for_result()

        return


if __name__ == "__main__":
    rospy.init_node("main_controller")
    is_debug = rospy.get_param("vision_project/is_debug", default=True)

    resources_directory = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources'
    database_file_name = os.path.join(resources_directory, 'long_term_interaction_state_db.json')

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

    main_controller = MainController(
        mongodb_statedb=state_database,
    )

    rospy.spin()
