import datetime

from interaction_engine.messager.directed_graph import DirectedGraph
from interaction_engine.messager.node import Node
from interaction_engine.text_populator import TextPopulator
from interaction_engine.text_populator import DatabasePopulator
from interaction_engine.text_populator import VarietyPopulator


class InteractionBuilder:
    class Graphs:
        DEMO = "demo interaction"

        POSSIBLE_GRAPHS = [
            DEMO
        ]

    def __init__(
            self,
            interaction_dict,
            variations_files,
            statedb,
    ):
        self._interaction_dict = interaction_dict
        self._variations_files = variations_files

        self._statedb = statedb

        self._database_populator = DatabasePopulator(self._statedb)
        self._variety_populator = VarietyPopulator(self._variations_files)

        self._text_populator = TextPopulator(self._variety_populator, self._database_populator)
        self._interactions = {}
        for graph_name in InteractionBuilder.Graphs.POSSIBLE_GRAPHS:
            self._interactions[graph_name] = self.build_graph_from_dict(
                self._interaction_dict,
                graph_name,
                self._text_populator,
            )

        self._possible_graphs = [graph for graph in self._interactions.values()]

    def build_graph_from_dict(
            self,
            interactions_dict,
            graph_name,
            text_populator=None,
            speaking_rate=None
    ):
        graph_dict = interactions_dict[graph_name]
        start_node_name = graph_dict["start_node_name"]
        nodes = []

        valid_speaking_rates = [
            "x-slow",
            "slow",
            "medium",
            "fast",
            "x-fast"
        ]
        if speaking_rate is not None and speaking_rate not in valid_speaking_rates:
            raise ValueError("Not a valid speaking rate")

        all_nodes_in_graph = graph_dict["nodes"]
        for node_name in all_nodes_in_graph.keys():
            node_info = all_nodes_in_graph[node_name]
            optional_values = {
                "args": None,
                "result_convert_from_str_fn": str,
                "result_db_key": None,
                "is_append_result": False,
                "tests": None,
                "is_confirm": False,
                "error_message": "Please enter a valid input",
                "error_options": ("Okay", "Oops")
            }
            for value in optional_values.keys():
                if value not in node_info:
                    node_info[value] = optional_values[value]

            if speaking_rate is not None:
                node_info["content"] = "<prosody rate=\"{speaking_rate}\">".format(speaking_rate=speaking_rate) + \
                                       node_info["content"] + "</prosody> "

            node = Node(
                name=node_name,
                transitions=node_info["transitions"],
                content=node_info["content"],
                options=node_info["options"],
                message_type=node_info["message_type"],
                args=node_info["args"],
                result_convert_from_str_fn=node_info["result_convert_from_str_fn"],
                result_db_key=node_info["result_db_key"],
                is_append_result=node_info["is_append_result"],
                tests=node_info["tests"],
                is_confirm=node_info["is_confirm"],
                error_message=node_info["error_message"],
                error_options=node_info["error_options"],
                text_populator=text_populator
            )
            nodes.append(node)

        return DirectedGraph(
            name=graph_name,
            nodes=nodes,
            start_node=start_node_name
        )

    @property
    def interactions(self):
        return self._interactions

    @property
    def possible_graphs(self):
        return self._possible_graphs
