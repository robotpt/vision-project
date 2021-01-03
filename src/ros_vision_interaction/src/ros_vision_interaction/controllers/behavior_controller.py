#!/usr/bin/python3.8
import actionlib
import datetime
import json
import logging
import os
import pymongo
import rospy
import schedule

from interaction_engine.engine import InteractionEngine
from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.json_database import Database
from interaction_engine.messager import Message, Node, DirectedGraph
from interaction_engine.planner import MessagerPlanner
from interaction_engine.text_populator import DatabasePopulator, VarietyPopulator, TextPopulator
from interfaces import CordialInterface
from mongodb_statedb import StateDb
from ros_vision_interaction.msg import StartInteractionAction, StartInteractionFeedback, StartInteractionResult

logging.basicConfig(level=logging.INFO)

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class BehaviorController:
    class Interactions:
        FIRST_INTERACTION = "first interaction"
        SCHEDULED_INTERACTION = "scheduled interaction"
        PROMPTED_INTERACTION = "prompted interaction"
        READING_EVALUATION = "reading evaluation"

        POSSIBLE_INTERACTIONS = [
            FIRST_INTERACTION,
            SCHEDULED_INTERACTION,
            PROMPTED_INTERACTION,
            READING_EVALUATION,
        ]

    def __init__(
            self,
            interactions_json_file,
            mongodb_statedb,
            text_populator=None,
            interface=None,
            start_interaction_action_name=START_INTERACTION_ACTION_NAME,
    ):
        self._state_database = mongodb_statedb

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        # set up action server
        self._start_interaction_action_server = actionlib.SimpleActionServer(
            start_interaction_action_name,
            StartInteractionAction,
            self.run_interaction_once,
            auto_start=False
        )
        self._start_interaction_action_server.register_preempt_callback(self._preempt_callback)

        self._text_populator = text_populator

        self._graphs_dict = self._build_all_possible_graphs_from_file(interactions_json_file)
        self._planner = MessagerPlanner([p for p in self._graphs_dict.values()])

        self._is_debug = rospy.get_param(
            "controllers/is_debug",
            False
        )

        self._start_interaction_action_server.start()

    # creates a dictionary of graph name: graph from a single json file
    def _build_all_possible_graphs_from_file(self, interactions_json_file):
        interaction_dict = {}
        with open(interactions_json_file) as f:
            interaction_setup_dict = json.load(f)
        for graph_name in interaction_setup_dict:
            interaction_dict[graph_name] = self._build_graph_from_json(graph_name, interaction_setup_dict[graph_name])

        return interaction_dict

    # builds a single DirectedGraph from a dictionary defined in the interaction json file
    def _build_graph_from_json(self, graph_name, graph_dict):
        start_node_name = graph_dict["start_node_name"]
        nodes = []

        all_nodes_info = graph_dict["nodes"]
        for node_name in all_nodes_info.keys():
            node_info = all_nodes_info[node_name]
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
            name=graph_name,
            nodes=nodes,
            start_node=start_node_name
        )

    # start interaction action callback
    def run_interaction_once(self, goal):
        interaction_type = goal.type
        if interaction_type not in BehaviorController.Interactions.POSSIBLE_INTERACTIONS:
            raise ValueError("Not a valid interaction type")

        result = StartInteractionResult()

        if not self._is_debug:
            engine = InteractionEngine(
                self._interface,
                self._planner,
                list(self._graphs_dict.values())
            )
            engine.run()
            rospy.loginfo("Interaction finished")
        else:
            seconds_to_sleep_for_tests = 3
            rospy.sleep(seconds_to_sleep_for_tests)
            result.is_interaction_successful = True

        if not self._start_interaction_action_server.is_preempt_requested():
            rospy.loginfo("Setting goal as succeeded")
            self._start_interaction_action_server.set_succeeded(result)

    def _build_first_interaction(self, planner):
        rospy.loginfo("Building first interaction")
        return planner

    def _build_prompted_interaction(self, planner):
        rospy.loginfo("Building prompted interaction")
        return planner

    def _build_scheduled_interaction(self, planner):
        rospy.loginfo("Building scheduled interaction")
        return planner

    def _preempt_callback(self):
        rospy.loginfo("Preempt requested for interaction server")
        self._start_interaction_action_server.set_preempted()


if __name__ == "__main__":
    rospy.init_node("behavior_controller")

    resources_directory = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources'
    interactions_json_file_name = os.path.join(resources_directory, 'long_term_interaction_nodes.json')
    variation_file_name = os.path.join(resources_directory, 'variations.json')

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

    database_populator = DatabasePopulator(database=state_database)
    variety_populator = VarietyPopulator(files=variation_file_name)
    text_populator = TextPopulator(variety_populator, database_populator)
    interface = CordialInterface(state_database)

    behavior_controller = BehaviorController(
        interactions_json_file=interactions_json_file_name,
        mongodb_statedb=state_database,
        text_populator=text_populator,
        interface=interface
    )

    rospy.spin()
