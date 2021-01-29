#!/usr/bin/python3.8
import actionlib
import datetime
import json
import pymongo
import rospy
import schedule

from controllers import InteractionManager
from interaction_builder import InteractionBuilder
from interfaces import CordialInterface
from vision_project_tools import init_db
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

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
            state_database,
            param_database,
            is_go_to_sleep_topic='cordial/sleep',
            screen_tap_topic='cordial/gui/event/mouse',
            start_interaction_action_name=START_INTERACTION_ACTION_NAME,
    ):
        self._interaction_manager = interaction_manager
        self._state_database = state_database
        self._param_database = param_database
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
            if self._is_in_scheduled_time_window():
                interaction = "scheduled interaction"
            else:
                interaction = "prompted interaction"
            self.run_manager_once(StartInteractionGoal(interaction))

    def _is_in_scheduled_time_window(self):
        current_time = datetime.datetime.now()
        checkin_window = datetime.timedelta(minutes=self._param_database["time window for checkin"])
        start_time = self._state_database.get("next checkin time") - checkin_window
        end_time = self._state_database.get("next checkin time") + checkin_window
        return start_time < current_time < end_time

    @property
    def is_engine_running(self):
        return self._is_engine_running


if __name__ == "__main__":

    rospy.init_node("interaction_manager")

    seconds_until_timeout = rospy.get_param("vision-project/seconds_until_interaction_timeout")

    DATABASE_NAME = "vision-project"
    host = rospy.get_param("mongodb/host")
    port = rospy.get_param("mongodb/port")
    state_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=DATABASE_NAME,
        collection_name="state_db"
    )
    state_db_key_values = {
        "first interaction time": None,
        "last interaction time": None,
        "next checkin time": None,
        "user name": None
    }
    init_db(state_database, state_db_key_values)

    param_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=DATABASE_NAME,
        collection_name="param_db"
    )
    param_db_keys = {
        "minutes between demo interactions": 5,
        # add units
        "time window for checkin": 15
    }

    init_db(param_database, param_db_keys)

    demo_interaction_file = rospy.get_param("vision-project/resources/demo/interactions")
    deployment_interaction_file = rospy.get_param("vision-project/resources/deployment/test_interactions")

    demo_variations_file = rospy.get_param("vision-project/resources/demo/variations")
    deployment_variations_file = rospy.get_param("vision-project/resources/deployment/variations")

    with open(demo_interaction_file) as f:
        demo_interaction_dict = json.load(f)
    with open(deployment_interaction_file) as f:
        deployment_interaction_dict = json.load(f)

    interface = CordialInterface(
        state_database,
        seconds_until_timeout=seconds_until_timeout
    )

    interaction_builder = InteractionBuilder(
        demo_interaction_dict=demo_interaction_dict,
        demo_variations_file=demo_variations_file,
        deployment_interaction_dict=deployment_interaction_dict,
        deployment_variations_file=deployment_variations_file,
        statedb=state_database
    )

    interaction_manager = InteractionManager(
        statedb=state_database,
        paramdb=param_database,
        interaction_builder=interaction_builder,
        interface=interface
    )

    ros_interaction_manager = RosInteractionManager(
        interaction_manager,
        state_database,
        param_database
    )

    rospy.spin()
