#!/usr/bin/env python

import datetime
import logging
import os
import random
import rospy
import schedule

from interaction_engine.cordial_interface import CordialInterface
from interaction_engine.database import Database
from interaction_engine.int_engine import InteractionEngine
from interaction_engine.message import Message
from interaction_engine.state import State
from interaction_engine.state_collection import StateCollection

from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal, MouseEvent
from std_msgs.msg import Bool


logging.basicConfig(level=logging.INFO)


class Keys:
    GREETING = "greeting"
    HOW_ARE_YOU = "how are you"
    TAKE_CARE = "take care"
    WHEN_TO_TALK = "when to talk"


greeting = State(
    name=Keys.GREETING,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="Hello!",
    next_states=[Keys.HOW_ARE_YOU],
    transitions={"Hello!": Keys.HOW_ARE_YOU, "Hi!": Keys.HOW_ARE_YOU}
)

how_are_you = State(
    name=Keys.HOW_ARE_YOU,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="How are you doing today?",
    next_states=[Keys.TAKE_CARE],
    transitions={
        "Pretty good.": Keys.TAKE_CARE,
        "Great!": Keys.TAKE_CARE,
        "Not too good.": Keys.TAKE_CARE
    }
)

take_care = State(
    name=Keys.TAKE_CARE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="Don't forget to drink enough water and get enough sleep!",
    next_states=[Keys.WHEN_TO_TALK],
    transitions={"Next": Keys.WHEN_TO_TALK}
)

when_to_talk = State(
    name=Keys.WHEN_TO_TALK,
    message_type=Message.Type.TIME_ENTRY,
    content="When would you like to talk tomorrow?",
    next_states=["exit"],
    args=["15", "15:15"]
)

state_collection = StateCollection(
    name="example interaction",
    init_state_name=Keys.WHEN_TO_TALK,
    states=[
        greeting,
        how_are_you,
        take_care,
        when_to_talk
    ]
)

cwd = os.getcwd()
database_file = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "example_interaction_database.json"
)

default_database_keys = [
    Keys.GREETING,
    Keys.HOW_ARE_YOU,
    Keys.TAKE_CARE,
    Keys.WHEN_TO_TALK
]

database_manager = Database(
    database_file=database_file,
    default_database_keys=default_database_keys
)

interface = CordialInterface(
    action_name="cordial/say_and_ask_on_gui",
    seconds_until_timeout=None
)

interaction_engine = InteractionEngine(
            state_collection=state_collection,
            database_manager=database_manager,
            interface=interface
        )

if __name__ == "__main__":

    while not rospy.is_shutdown():
        rospy.logdebug("Scheduled interaction running")
        interaction_engine.run()
        rospy.sleep(5)
