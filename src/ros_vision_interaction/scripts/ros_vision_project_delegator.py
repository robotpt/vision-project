#!/usr/bin/env python3.8
import actionlib
import datetime
import pymongo
import rospy
import schedule

from controllers import VisionProjectDelegator
from vision_project_tools import init_db
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

from ros_vision_interaction.msg import StartInteractionAction, StartInteractionGoal
from std_msgs.msg import Bool

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class RosVisionProjectDelegator:

    def __init__(
            self,
            vision_project_delegator,
            is_record_interaction_topic='data_capture/is_record_interaction',
            is_record_evaluation_topic='data_capture/is_record_evaluation',
            start_interaction_action_name=START_INTERACTION_ACTION_NAME,
    ):
        self._delegator = vision_project_delegator
        self._state_database = state_database
        self._seconds_between_updates = rospy.get_param("vision-project/controllers/seconds_between_updates")

        # action client to start interaction
        self._start_interaction_client = actionlib.SimpleActionClient(
            start_interaction_action_name,
            StartInteractionAction
        )

        # ROS publishers and subscribers
        self._is_record_interaction_publisher = rospy.Publisher(is_record_interaction_topic, Bool, queue_size=1)
        self._is_record_evaluation_publisher = rospy.Publisher(is_record_evaluation_topic, Bool, queue_size=1)

        # update scheduler
        self._scheduler = schedule.Scheduler()
        self._scheduler.every(self._seconds_between_updates).seconds.do(self.update)

        self._is_debug = rospy.get_param(
            "vision-project/controllers/is_debug",
            False
        )

    def run_scheduler_once(self):
        self._scheduler.run_pending()

    def update(self):
        rospy.loginfo("Running update")
        self._delegator.update()
        interaction_type = self._delegator.determine_interaction_type()
        if interaction_type is not None:
            rospy.loginfo("Delegating interaction: {}".format(interaction_type))
            self.delegate_interaction(interaction_type)

    def delegate_interaction(self, interaction_type):
        self._start_interaction_client.wait_for_server()

        start_interaction_goal = StartInteractionGoal()
        start_interaction_goal.type = interaction_type

        if not self._is_debug:
            # TODO: add a feedback callback
            rospy.loginfo("Sending goal to start interaction")
            self._start_interaction_client.send_goal(start_interaction_goal)
            self._start_interaction_client.wait_for_result()
        return


if __name__ == "__main__":

    rospy.init_node("vision_project_delegator")

    is_run_demo_interaction = rospy.get_param("vision-project/is_run_demo_interaction")

    DATABASE_NAME = "vision-project"
    host = rospy.get_param("mongodb/host")
    port = rospy.get_param("mongodb/port")
    state_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=DATABASE_NAME,
        collection_name="state_db"
    )
    state_db_key_values = {
        "evaluation start time": None,
        "evaluation stop time": None,
        "first interaction time": None,
        "good to chat": None,
        "is done evaluation today": False,
        "is off checkin": None,
        "is run prompted": False,
        "last interaction time": None,
        "last update datetime"
        "next checkin datetime": None,
        "reading performance": {},  # the keys will be datetimes, and the values will be reading speed in WPM (float)
        "recording start time": None,
        "user name": None,
    }
    init_db(state_database, state_db_key_values)

    checkin_window_seconds = rospy.get_param("vision-project/controllers/seconds_between_updates")

    vision_project_delegator = VisionProjectDelegator(
        statedb=state_database,
        checkin_window_seconds=checkin_window_seconds,
        is_run_demo_interaction=is_run_demo_interaction
    )

    ros_vision_project_delegator = RosVisionProjectDelegator(vision_project_delegator)

    while not rospy.is_shutdown():
        ros_vision_project_delegator.run_scheduler_once()
        rospy.sleep(1)
