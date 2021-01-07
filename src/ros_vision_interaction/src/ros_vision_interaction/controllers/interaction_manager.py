#!/usr/bin/python3.8
import datetime
import json
import logging
import os
import rospy
import schedule

from interaction_engine.engine import InteractionEngine
from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.messager import Message, Node, DirectedGraph
from interaction_engine.planner import MessagerPlanner

logging.basicConfig(level=logging.INFO)


class InteractionManager:
    class Interactions:
        FIRST_INTERACTION = "first interaction"
        PROMPTED_INTERACTION = "prompted interaction"
        SCHEDULED_INTERACTION = "scheduled interaction"
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
            interface=None
    ):
        self._state_database = mongodb_statedb

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        self._text_populator = text_populator

        self._graphs_dict = self._build_all_possible_graphs_from_file(interactions_json_file)
        self._planner = MessagerPlanner([p for p in self._graphs_dict.values()])

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

    def run_interaction_once(self, interaction_type):
        if interaction_type not in InteractionManager.Interactions.POSSIBLE_INTERACTIONS:
            raise ValueError("Not a valid interaction type")

        self.build_interaction(interaction_type, self._planner)
        engine = InteractionEngine(
            self._interface,
            self._planner,
            list(self._graphs_dict.values())
        )
        engine.run()

    def build_interaction(self, interaction_type, planner):
        build_interaction_dict = {
            InteractionManager.Interactions.FIRST_INTERACTION: self._build_first_interaction,
            InteractionManager.Interactions.PROMPTED_INTERACTION: self._build_prompted_interaction,
            InteractionManager.Interactions.SCHEDULED_INTERACTION: self._build_scheduled_interaction,
            InteractionManager.Interactions.READING_EVALUATION:
                lambda planner: planner.insert(self._graphs_dict["reading evaluation"]),
        }

        return build_interaction_dict[interaction_type](planner)

    def _build_first_interaction(self, planner):
        rospy.loginfo("Building first interaction")
        planner.insert(self._graphs_dict["introduce QT"])
        planner.insert(self._graphs_dict["first interaction"])
        return planner

    def _build_prompted_interaction(self, planner):
        rospy.loginfo("Building prompted interaction")
        planner.insert(self._graphs_dict["greeting"])
        planner.insert(self._graphs_dict["prompted interaction"])
        return planner

    def _build_scheduled_interaction(self, planner):
        rospy.loginfo("Building scheduled interaction")
        planner.insert(self._graphs_dict["greeting"])
        planner.insert(self._graphs_dict["scheduled interaction"])
        return planner

