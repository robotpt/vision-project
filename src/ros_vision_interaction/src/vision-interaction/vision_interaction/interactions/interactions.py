import json
import logging
import os

from interaction_engine.messager import Message, Node, DirectedGraph

from data_structures import text_populator

resources_directory = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources/deployment/'
INTERACTIONS_FILE_NAME = os.path.join(resources_directory, 'deployment_interactions.json')
VARIATIONS_FILE_NAME = os.path.join(resources_directory, 'variations.json')


with open(INTERACTIONS_FILE_NAME) as f:
    INTERACTIONS_DICT = json.load(f)


def build_graph_from_json(interactions_dict, graph_name):
    graph_dict = interactions_dict[graph_name]
    start_node_name = graph_dict["start_node_name"]
    nodes = []

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
            text_populator=text_populator
        )
        nodes.append(node)

    return DirectedGraph(
        name=graph_name,
        nodes=nodes,
        start_node=start_node_name
    )


class Interactions:

    FIRST_INTERACTION = build_graph_from_json(INTERACTIONS_DICT, "first interaction")
    GREETING = build_graph_from_json(INTERACTIONS_DICT, "greeting")
    INTRODUCE_QT = build_graph_from_json(INTERACTIONS_DICT, "introduce QT")
    PROMPTED_INTERACTION = build_graph_from_json(INTERACTIONS_DICT, "prompted interaction")
    READING_EVALUATION = build_graph_from_json(INTERACTIONS_DICT, "reading evaluation")
    SCHEDULED_INTERACTION = build_graph_from_json(INTERACTIONS_DICT, "scheduled interaction")

    POSSIBLE_GRAPHS = [
        FIRST_INTERACTION,
        GREETING,
        INTRODUCE_QT,
        PROMPTED_INTERACTION,
        READING_EVALUATION,
        SCHEDULED_INTERACTION
    ]
