import datetime
import json
import logging
import schedule

from interaction_engine.engine import InteractionEngine
from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.json_database import Database
from interaction_engine.messager import Message, Node, DirectedGraph
from interaction_engine.planner import MessagerPlanner
from interaction_engine.text_populator import DatabasePopulator, VarietyPopulator, TextPopulator

logging.basicConfig(level=logging.INFO)


class BehaviorController:
    class Interactions:
        FIRST_INTERACTION = "first interaction"
        SCHEDULED_INTERACTION = "scheduled interaction"
        PROMPTED_INTERACTION = "prompted interaction"

    def __init__(
            self,
            interactions_json_file,
            state_database_file,
            variations_json_file=None,
            interface=None,
    ):
        self._state_database = Database(database_file_name=state_database_file)
        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        self._build_interaction_dict = {
            BehaviorController.Interactions.FIRST_INTERACTION: self._build_first_interaction,
            BehaviorController.Interactions.PROMPTED_INTERACTION: self._build_prompted_interaction,
            BehaviorController.Interactions.SCHEDULED_INTERACTION: self._build_scheduled_interaction,
        }
        self._graphs_dict = self._build_all_possible_graphs_from_file(interactions_json_file)
        self._planner = MessagerPlanner([p for p in self._graphs_dict.values()])

        self._database_populator = DatabasePopulator(database=self._state_database)
        self._variety_populator = VarietyPopulator(files=variations_json_file)
        self._text_populator = TextPopulator(self._variety_populator, self._database_populator)

    def _build_all_possible_graphs_from_file(self, interactions_json_file):
        interaction_dict = {}
        with open(interactions_json_file) as f:
            interaction_setup_dict = json.load(f)
        for graph_name in interaction_setup_dict:
            interaction_dict[graph_name] = self._build_graph_from_json(interaction_setup_dict[graph_name])

        return interaction_dict

    def _build_graph_from_json(self, graph_dict):
        graph_name = graph_dict["graph_name"]
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
        if interaction_type not in self._build_interaction_dict.keys():
            raise ValueError("Not a valid interaction type")

        # call a helper method that inserts graphs into the planner
        # interaction = self._build_interaction_dict[interaction_type](self._planner)
        # engine = InteractionEngine(
        #     self._possible_interaction_types,
        #     self._planner,
        #     interaction
        # )

    def _build_first_interaction(self, planner):
        pass

    def _build_prompted_interaction(self, planner):
        pass

    def _build_scheduled_interaction(self, planner):
        pass
