#!/usr/bin/python3.8
import actionlib
import datetime
import json
import pymongo
import rospy

from controllers import InteractionManager
from interaction_builder import InteractionBuilder
from interfaces import CordialInterface
from vision_project_tools import init_db
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

from ros_vision_interaction.msg import StartInteractionAction, StartInteractionFeedback, StartInteractionResult
from std_msgs.msg import Bool

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class RosInteractionManager:

    def __init__(
            self,
            interaction_manager,
            state_database,
            is_go_to_sleep_topic='cordial/sleep',
            start_interaction_action_name=START_INTERACTION_ACTION_NAME,
    ):
        self._interaction_manager = interaction_manager
        self._state_database = state_database

        self._sleep_publisher = rospy.Publisher(is_go_to_sleep_topic, Bool, queue_size=1)

        # set up action server
        self._start_interaction_action_server = actionlib.SimpleActionServer(
            start_interaction_action_name,
            StartInteractionAction,
            self.run_manager_once,
            auto_start=False
        )
        self._start_interaction_action_server.register_preempt_callback(self._preempt_callback)

        self._is_debug = rospy.get_param(
            "vision-project/controllers/is_debug",
            False
        )
        self._start_interaction_action_server.start()

    # start interaction action callback
    def run_manager_once(self, goal):
        self._state_database.set("is interaction finished", False)
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

        self._state_database.set("last interaction datetime", datetime.datetime.now())

    def _preempt_callback(self):
        rospy.loginfo("Preempt requested for interaction server")
        self._start_interaction_action_server.set_preempted()


if __name__ == "__main__":

    rospy.init_node("interaction_manager")

    seconds_until_timeout = rospy.get_param("vision-project/params/seconds_until_interaction_timeout")

    DATABASE_NAME = "vision-project"
    host = rospy.get_param("mongodb/host")
    port = rospy.get_param("mongodb/port")
    state_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=DATABASE_NAME,
        collection_name="state_db"
    )
    state_db_key_values = {
        "average eval score": None,
        "current eval score": None,
        "first interaction datetime": None,
        "good time to talk": False,
        "is done eval today": False,
        "is done prompted today": False,
        "is done perseverance today": False,
        "is done mindfulness today": False,
        "is done goal setting today": False,
        "is interaction finished": False,
        "is prompted by user": False,
        "is run prompted content": False,
        "last eval score": None,
        "last interaction datetime": None,
        "last update datetime": None,
        "next checkin datetime": None,
        "num of days since last eval": 0,
        "num of days since last prompt": 0,
        "num of days since last perseverance": 0,
        "num of days since last mindfulness": 0,
        "num of days since last goal setting": 0,
    }
    init_db(state_database, state_db_key_values)

    deployment_interaction_file = rospy.get_param("vision-project/resources/deployment/test_interactions")

    interaction_variations_file = rospy.get_param("vision-project/resources/deployment/interaction-variations")
    grit_dialogue_variations = rospy.get_param("vision-project/resources/deployment/grit-dialogue")

    with open(deployment_interaction_file) as f:
        deployment_interaction_dict = json.load(f)

    interface = CordialInterface(
        state_database,
        seconds_until_timeout=seconds_until_timeout
    )

    interaction_builder = InteractionBuilder(
        interaction_dict=deployment_interaction_dict,
        variations_files=interaction_variations_file,
        statedb=state_database
    )

    interaction_manager = InteractionManager(
        statedb=state_database,
        interaction_builder=interaction_builder,
        interface=interface
    )

    ros_interaction_manager = RosInteractionManager(
        interaction_manager,
        state_database,
    )

    rospy.spin()
