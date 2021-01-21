from interaction_engine.messager.directed_graph import DirectedGraph
from interaction_engine.messager.node import Node
from interaction_engine.text_populator import TextPopulator
from interaction_engine.text_populator import DatabasePopulator
from interaction_engine.text_populator import VarietyPopulator


class InteractionBuilder:
    class Graphs:
        DEMO_INTERACTION = "demo interaction"
        FIRST_INTERACTION = "first interaction"
        GREETING = "greeting"
        INTRODUCE_QT = "introduce QT"
        PROMPTED_INTERACTION = "prompted interaction"
        READING_EVALUATION = "reading evaluation"
        SCHEDULED_INTERACTION = "scheduled interaction"

    def __init__(
            self,
            demo_interaction_dict,
            demo_variations_file,
            deployment_interaction_dict,
            deployment_variations_file,
            statedb
    ):
        self._demo_interaction_dict = demo_interaction_dict
        self._deployment_interaction_dict = deployment_interaction_dict
        self._demo_variations_file = demo_variations_file
        self._deployment_variations_file = deployment_variations_file

        self._statedb = statedb

        self._database_populator = DatabasePopulator(self._statedb)
        self._demo_variety_populator = VarietyPopulator(self._demo_variations_file)
        self._deployment_variety_populator = VarietyPopulator(self._deployment_variations_file)

        self._demo_text_populator = TextPopulator(self._demo_variety_populator, self._database_populator)
        self._deployment_text_populator = TextPopulator(self._deployment_variety_populator, self._database_populator)
        self._interactions = {
            InteractionBuilder.Graphs.DEMO_INTERACTION: self.build_graph_from_dict(
                self._demo_interaction_dict,
                InteractionBuilder.Graphs.DEMO_INTERACTION,
                self._demo_text_populator,
                speaking_rate="slow"
            ),
            InteractionBuilder.Graphs.FIRST_INTERACTION: self.build_graph_from_dict(
                self._deployment_interaction_dict,
                InteractionBuilder.Graphs.FIRST_INTERACTION,
                self._deployment_text_populator,
                speaking_rate="slow"
            ),
            InteractionBuilder.Graphs.GREETING: self.build_graph_from_dict(
                self._deployment_interaction_dict,
                InteractionBuilder.Graphs.GREETING,
                self._deployment_text_populator,
                speaking_rate="slow"
            ),
            InteractionBuilder.Graphs.INTRODUCE_QT: self.build_graph_from_dict(
                self._deployment_interaction_dict,
                InteractionBuilder.Graphs.INTRODUCE_QT,
                self._deployment_text_populator,
                speaking_rate="slow"
            ),
            InteractionBuilder.Graphs.PROMPTED_INTERACTION: self.build_graph_from_dict(
                self._deployment_interaction_dict,
                InteractionBuilder.Graphs.PROMPTED_INTERACTION,
                self._deployment_text_populator,
                speaking_rate="slow"
            ),
            InteractionBuilder.Graphs.READING_EVALUATION: self.build_graph_from_dict(
                self._deployment_interaction_dict,
                InteractionBuilder.Graphs.READING_EVALUATION,
                self._deployment_text_populator,
                speaking_rate="slow"
            ),
            InteractionBuilder.Graphs.SCHEDULED_INTERACTION: self.build_graph_from_dict(
                self._deployment_interaction_dict,
                InteractionBuilder.Graphs.SCHEDULED_INTERACTION,
                self._deployment_text_populator,
                speaking_rate="slow"
            ),
        }

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

            if speaking_rate is not None:
                content = "<prosody rate=\"{speaking_rate}\">".format(speaking_rate=speaking_rate) + \
                          node_info["content"] + "</prosody> "
            else:
                content = node_info["content"]

            node = Node(
                name=node_name,
                transitions=node_info["transitions"],
                content=content,
                options=node_info["options"],
                message_type=node_info["message_type"],
                args=node_info["args"],
                result_db_key=node_info["result_db_key"],
                is_append_result=node_info["is_append_result"],
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
