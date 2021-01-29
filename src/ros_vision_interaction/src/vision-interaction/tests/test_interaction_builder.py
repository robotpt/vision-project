import mock
import pytest


def test_build_graph_from_json(interaction_builder):
    test_graph_dict = {
        "test_directed_graph": {
            "start_node_name": "node1",
            "nodes": {
                "node1": {
                    "transitions": ["node1", "node2"],
                    "content": "node1 content",
                    "options": ["option1-1", "option1-2"],
                    "message_type": "multiple choice"
                },
                "node2": {
                    "transitions": ["node1", "exit"],
                    "content": "node2 content",
                    "options": ["option2-1", "option2-exit"],
                    "message_type": "multiple choice"
                },
            }
        }
    }
    graph_name = list(test_graph_dict.keys())[0]
    test_directed_graph = interaction_builder.build_graph_from_dict(test_graph_dict, graph_name)
    nodes_dict_from_graph = test_directed_graph._nodes_dict

    assert "node1" in nodes_dict_from_graph.keys()
    assert "node2" in nodes_dict_from_graph.keys()

    expected_nodes_dict = test_graph_dict["test_directed_graph"]["nodes"]
    for node in expected_nodes_dict:
        expected_transitions = expected_nodes_dict[node]["transitions"]
        actual_transitions = nodes_dict_from_graph[node].transitions
        assert expected_transitions == actual_transitions

        expected_content = expected_nodes_dict[node]["content"]
        actual_content = nodes_dict_from_graph[node].message.content
        assert expected_content == actual_content
