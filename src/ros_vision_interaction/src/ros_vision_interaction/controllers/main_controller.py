#!/usr/bin/env python
import actionlib
import datetime
import logging
import os
import rospy
import schedule

from interaction_engine.engine import InteractionEngine
from interaction_engine.json_database import Database
from controllers import BehaviorController
from interaction_engine.text_populator import DatabasePopulator, VarietyPopulator, TextPopulator

from cordial_msgs.msg import MouseEvent
from std_msgs.msg import Bool
from ros_vision_interaction.msg import StartInteractionAction, StartInteractionActionGoal

logging.basicConfig(level=logging.INFO)


class MainController:

    def __init__(
        self,
        state_database_file,
        is_go_to_sleep_topic='cordial/sleep',
        is_record_topic='data_capture/is_record',
        screen_tap_topic='cordial/gui/event/mouse',
    ):

        self._state_database = Database(database_file_name=state_database_file)
        self._set_initial_db_keys(self._state_database)

        # action client to start interaction
        self._start_interaction_client = actionlib.SimpleActionClient(
            self.delegate_interaction,
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

    def _update(self):
        pass

    def _set_initial_db_keys(self, db):
        pass

    def _screen_tap_listener_callback(self, _):
        self._is_start_interaction = True

    def delegate_interaction(self):
        # determine interaction type from scheduler/state db
        interaction_type = None


if __name__ == "__main__":

    resources_directory = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources'
    database_file_name = os.path.join(resources_directory, 'long_term_interaction_state_db.json')

    rospy.init_node("main_controller")
    main_controller = MainController(
        state_database_file=database_file_name
    )

    rospy.spin()
