#!/usr/bin/env python
import json
import logging
import os
import random
import rospy

from interaction_engine.cordial_interface import CordialInterface
from interaction_engine.database import Database
from interaction_engine.int_engine import InteractionEngine
from interaction_engine.state import State
from interaction_engine.state_collection import StateCollection

from cordial_msgs.msg import MouseEvent
from std_msgs.msg import Bool

logging.basicConfig(level=logging.INFO)


class DemoInteraction:

    def __init__(self, interaction_json_file, database_file_name, is_clear_database=True):

        self._interface = CordialInterface(
            action_name="cordial/say_and_ask_on_gui",
            seconds_until_timeout=None
        )

        with open(interaction_json_file) as f:
            interaction_setup_dict = json.load(f)
        self._jokes = interaction_setup_dict["jokes"]
        self._news_stories = interaction_setup_dict["news stories"]

        states_from_json = self._add_joke_and_news_content_to_states_dict(
            interaction_setup_dict["states"],
            interaction_setup_dict["joke prefaces"]
        )
        state_names = [str(state_name) for state_name in states_from_json.keys()]

        self._state_collection = StateCollection(
            name="demo interaction",
            init_state_name="ask to chat",
            states=states_from_json
        )
        self._database = Database(
            database_file_name=database_file_name,
            default_database_keys=state_names
        )
        self._is_record_publisher = rospy.Publisher("data_capture/is_record", Bool, queue_size=1)
        self._screen_tap_listener = rospy.Subscriber(
            "cordial/gui/event/mouse",
            MouseEvent,
            callback=self._screen_tap_listener_callback,
            queue_size=1
        )
        self._sleep_publisher = rospy.Publisher("cordial/sleep", Bool, queue_size=1)
        self._is_start_interaction = False

        if is_clear_database:
            self._database.clear_entire_database()

    def run_once(self):
        interaction_engine = InteractionEngine(
            state_collection=self._state_collection,
            database_manager=self._database,
            interface=self._interface
        )
        while not self._is_start_interaction:
            rospy.sleep(1)
        interaction_engine.run()
        self._is_start_interaction = False
        self._sleep_publisher.publish(True)

    def _add_joke_and_news_content_to_states_dict(self, states_from_json, joke_prefaces):
        state_names = states_from_json.keys()
        for state_name in state_names:
            if "joke" in str(states_from_json[state_name]):
                states_from_json[state_name]["content"] = self._make_random_joke_content(joke_prefaces)
            if str(states_from_json[state_name]) == "share news story":
                states_from_json[state_name]["content"] = str(random.choice(self._news_stories))
        return states_from_json

    def _make_random_joke_content(self, joke_prefaces):
        joke_category = str(random.choice(self._jokes.keys()))
        joke_content = str(random.choice(joke_prefaces).format(category=joke_category))
        joke = str(random.choice(self._jokes[joke_category]))
        joke_content += " <break time=\"1s\"/>" + joke
        self._jokes[joke_category].remove(joke)
        return joke_content

    def _screen_tap_listener_callback(self, _):
        self._is_start_interaction = True


if __name__ == "__main__":

    interaction_json_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "sar_demo_states.json"
    )
    database_file_name = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "sar_demo.json"
    )

    demo_interaction = DemoInteraction(interaction_json_file, database_file_name)

    while not rospy.is_shutdown():
        demo_interaction.run_once()

