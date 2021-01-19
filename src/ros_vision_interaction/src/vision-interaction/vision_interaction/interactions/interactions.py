import json
import os
# import rospy

from data_structures import demo_resources_directory, \
    deployment_resources_directory, \
    demo_text_populator, \
    deployment_text_populator
from interaction_engine.messager import Message, Node, DirectedGraph

# deployment_resources_directory = rospy.get_param('vision-project/resources/path/deployment')
DEPLOYMENT_INTERACTIONS_FILE_NAME = os.path.join(deployment_resources_directory, 'deployment_interactions.json')
DEPLOYMENT_VARIATIONS_FILE_NAME = os.path.join(deployment_resources_directory, 'variations.json')

# demo_resources_directory = rospy.get_param('vision-project/resources/path/demo')
DEMO_INTERACTIONS_FILE_NAME = os.path.join(demo_resources_directory, 'demo_interaction.json')
DEMO_VARIATIONS_FILE_NAME = os.path.join(demo_resources_directory, 'variations.json')

with open(DEPLOYMENT_INTERACTIONS_FILE_NAME) as f:
    DEPLOYMENT_INTERACTIONS_DICT = json.load(f)

with open(DEMO_INTERACTIONS_FILE_NAME) as f:
    DEMO_INTERACTIONS_DICT = json.load(f)


def build_graph_from_json(
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


class Interactions:
    DEMO_INTERACTION = build_graph_from_json(
        DEMO_INTERACTIONS_DICT,
        "demo interaction",
        demo_text_populator,
        speaking_rate="slow"
    )
    FIRST_INTERACTION = build_graph_from_json(
        DEPLOYMENT_INTERACTIONS_DICT,
        "first interaction",
        deployment_text_populator
    )
    GREETING = build_graph_from_json(
        DEPLOYMENT_INTERACTIONS_DICT,
        "greeting",
        deployment_text_populator
    )
    INTRODUCE_QT = build_graph_from_json(
        DEPLOYMENT_INTERACTIONS_DICT,
        "introduce QT",
        deployment_text_populator
    )
    PROMPTED_INTERACTION = build_graph_from_json(
        DEPLOYMENT_INTERACTIONS_DICT,
        "prompted interaction",
        deployment_text_populator
    )
    READING_EVALUATION = build_graph_from_json(
        DEPLOYMENT_INTERACTIONS_DICT,
        "reading evaluation",
        deployment_text_populator
    )
    SCHEDULED_INTERACTION = build_graph_from_json(
        DEPLOYMENT_INTERACTIONS_DICT,
        "scheduled interaction",
        deployment_text_populator
    )

    POSSIBLE_GRAPHS = [
        DEMO_INTERACTION,
        FIRST_INTERACTION,
        GREETING,
        INTRODUCE_QT,
        PROMPTED_INTERACTION,
        READING_EVALUATION,
        SCHEDULED_INTERACTION
    ]
