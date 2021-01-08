#!/usr/bin/env python
import datetime
import json
import logging
import rospy

from interaction_engine.engine import InteractionEngine
from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.json_database import Database
from interaction_engine.messager import Node, DirectedGraph
from interaction_engine.planner import MessagerPlanner
from interaction_engine.text_populator import DatabasePopulator, VarietyPopulator, TextPopulator

from cordial_msgs.msg import MouseEvent
from std_msgs.msg import Bool

logging.basicConfig(level=logging.INFO)


class DemoInteractionController:

    def __init__(
        self,
        interaction_json_file,
        start_node_name,
        database_file_name,
        variation_file_name=None,
        is_go_to_sleep_topic='cordial/sleep',
        is_record_topic='data_capture/is_record',
        screen_tap_topic='cordial/gui/event/mouse',
        interface=None,
        is_clear_database=True
    ):
        with open(interaction_json_file) as f:
            interaction_setup_dict = json.load(f)

        self._start_node_name = start_node_name

        self._database = Database(database_file_name=database_file_name)
        self._database.database = self._create_initial_db(interaction_setup_dict)

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._database)
        self._interface = interface

        self._database_populator = DatabasePopulator(database=self._database)
        self._variety_populator = VarietyPopulator(files=variation_file_name)
        self._text_populator = TextPopulator(self._variety_populator, self._database_populator)

        self._interaction = [self._build_graph_from_dict(interaction_setup_dict)]
        self._plan = MessagerPlanner(self._interaction)
        self._start_node_name = start_node_name

        self._is_record_publisher = rospy.Publisher(is_record_topic, Bool, queue_size=1)
        self._screen_tap_listener = rospy.Subscriber(
            screen_tap_topic,
            MouseEvent,
            callback=self._screen_tap_listener_callback,
            queue_size=1
        )
        self._sleep_publisher = rospy.Publisher(is_go_to_sleep_topic, Bool, queue_size=1)
        self._is_start_interaction = False

        self._is_clear_database = is_clear_database
        if self._is_clear_database:
            self._database.clear_entire_database()

    def run_once(self):
        self._plan.insert(self._interaction)
        interaction_engine = InteractionEngine(
            interface=self._interface,
            plan=self._plan,
            messagers=self._interaction
        )
        self._is_record_publisher.publish(True)
        interaction_engine.run()
        self._is_start_interaction = False
        self._is_record_publisher.publish(False)
        self._sleep_publisher.publish(True)

        if self._is_clear_database:
            self._database.clear_entire_database()

    def _build_graph_from_dict(self, dict_from_json_file):
        nodes = []
        for node_name in dict_from_json_file:
            node_info = dict_from_json_file[node_name]
            optional_values = [
                "args",
                "result_convert_from_str_fn",
                "result_db_key",
                "is_append_result",
                "is_confirm",
                "error_message",
                "error_options"
            ]
            for value in optional_values:
                if value not in node_info:
                    node_info[value] = None

            node = Node(
                name=node_name,
                transitions=node_info["transitions"],
                content=node_info["content"],
                options=node_info["options"],
                message_type=node_info["message_type"],
                args=node_info["args"],
                result_db_key=node_info["result_db_key"],
                is_append_result=node_info["is_append_result"],
                is_confirm=node_info["is_confirm"],
                error_message=node_info["error_message"],
                error_options=node_info["error_options"],
                text_populator=self._text_populator
            )
            nodes.append(node)
        return DirectedGraph(
            name="demo interaction",
            nodes=nodes,
            start_node=self._start_node_name
        )

    def _create_initial_db(self, interaction_setup_dict):
        database_keys = []
        for node_name in interaction_setup_dict.keys():
            if "result_db_key" in interaction_setup_dict[node_name]:
                database_keys.append(interaction_setup_dict[node_name]["result_db_key"])
        return {key: "" for key in database_keys}

    def _screen_tap_listener_callback(self, _):
        self._is_start_interaction = True
