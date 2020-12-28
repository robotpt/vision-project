import datetime
import logging
import rospy
import schedule

from interaction_engine.engine import InteractionEngine
from interaction_engine.json_database import Database
from controllers import BehaviorController
from interaction_engine.text_populator import DatabasePopulator, VarietyPopulator, TextPopulator

from cordial_msgs.msg import MouseEvent
from std_msgs.msg import Bool

logging.basicConfig(level=logging.INFO)


class MainController:

    def __init__(
        self,
        behavior_controller,
        state_database_file,
        is_go_to_sleep_topic='cordial/sleep',
        is_record_topic='data_capture/is_record',
        screen_tap_topic='cordial/gui/event/mouse',
    ):
        self._behavior_controller = behavior_controller

        self._state_database = Database(database_file_name=state_database_file)
        self._set_initial_db_keys(self._state_database)

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
        self._scheduled_interaction_scheduler = schedule.Scheduler()

    def _update(self):
        pass

    def _set_initial_db_keys(self, db):
        pass

    def _screen_tap_listener_callback(self, _):
        self._is_start_interaction = True

    def _run_interaction_once(self):
        # determine interaction type from scheduler/state db
        interaction_type = None
        self._behavior_controller.run_interaction_once(interaction_type)
