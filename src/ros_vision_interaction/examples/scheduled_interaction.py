#!/usr/bin/env python

import datetime
import logging
import os
import random
import rospy
import schedule

from scheduled_interaction_states import \
    first_interaction, how_are_you_interaction, check_in_interaction, weekend_interaction, database_keys, Keys

from interaction_engine.database import Database
from interaction_engine.int_engine import InteractionEngine
from interaction_engine.cordial_interface import CordialInterface

from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal, MouseEvent
from std_msgs.msg import Bool


logging.basicConfig(level=logging.INFO)


class ScheduledInteraction:

    def __init__(self, database_file):
        self._interface = CordialInterface(
            action_name="cordial/say_and_ask_on_gui",
            seconds_until_timeout=None
        )
        self._database = Database(database_file, default_database_keys=database_keys)
        self._interaction_plan = []

        self._interaction_scheduler = schedule.Scheduler()
        self._sleep_publisher = rospy.Publisher("cordial/sleep", Bool, queue_size=1)

    def run_once(self):
        self._choose_interaction()
        for interaction in self._interaction_plan:
            interaction_engine = InteractionEngine(interaction, self._database, self._interface)
            interaction_engine.run()
        self._sleep_publisher.publish(Bool(data=True))

    def _choose_interaction(self):
        if self._database[Keys.IS_FIRST_INTERACTION] == "":
            self._database[Keys.IS_FIRST_INTERACTION] = "false"
            self._interaction_plan.append(first_interaction)
        if datetime.datetime.today().weekday() >= 5:
            self._interaction_plan.append(weekend_interaction)
        else:
            self._interaction_plan.append(random.choice([how_are_you_interaction, check_in_interaction]))

    def clear_database(self):
        self._database.clear_entire_database()


if __name__ == "__main__":

    cwd = os.getcwd()
    database_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "scheduled_interaction_database.json"
    )

    example_interaction = ScheduledInteraction(database_file)
    example_interaction.clear_database()

    while not rospy.is_shutdown():
        rospy.logdebug("Scheduled interaction running")
        example_interaction.run_once()
        rospy.sleep(1.)
