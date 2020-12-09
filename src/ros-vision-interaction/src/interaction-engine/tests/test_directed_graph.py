import unittest
from interaction_engine.messager.directed_graph import DirectedGraph
from interaction_engine.messager.node import Node
from interaction_engine.messager import Message

content = 'Question'
options = ['left', 'right']


message_kwargs = {
    'content': content,
    'options': options,
    'message_type': Message.Type.MULTIPLE_CHOICE,
}


def left_option():
    return options[0]


def right_option():
    return options[1]


node1 = Node(
    name='node1',
    transitions=['node2', 'node1'],
    **message_kwargs,
)
node2 = Node(
    name='node2',
    transitions=['node1', 'node3'],
    **message_kwargs,
)
node3 = Node(
    name='node3',
    transitions=['exit', 'exit'],
    **message_kwargs,
)


class TestDirectedGraph(unittest.TestCase):

    def test_step_through_nodes(self) -> None:
        nodes_ = [node1, node2, node3]
        directed_graph = DirectedGraph(
            name='graph1',
            nodes=nodes_,
            start_node='node1'
        )

        for _ in range(10):
            self.assertEqual(
                Message(**message_kwargs).content,
                directed_graph.get_message().content
            )

            for _ in range(10):
                self.assertTrue(directed_graph.is_active)
                directed_graph.transition(right_option())
                self.assertEqual(
                    node1.name,
                    directed_graph.current_node,
                )

            self.assertTrue(directed_graph.is_active)
            directed_graph.transition(left_option())

            for _ in range(10):
                directed_graph.transition(left_option())
                directed_graph.transition(left_option())
                self.assertEqual(
                    node2.name,
                    directed_graph.current_node,
                )
                self.assertTrue(directed_graph.is_active)

            directed_graph.transition(right_option())
            self.assertTrue(directed_graph.is_active)
            directed_graph.transition(left_option())
            self.assertFalse(directed_graph.is_active)

            for _ in range(10):
                self.assertRaises(
                    RuntimeError,
                    directed_graph.transition,
                    left_option()
                )
                self.assertIsNone(
                    directed_graph.current_node
                )

            directed_graph.reset()

    def test_bad_node_transitions(self):

        # transition to a nonexistent node
        self.assertRaises(
            ValueError,
            lambda: DirectedGraph(
                name="Foo",
                start_node="node1",
                nodes=[
                    Node(
                        name='node1',
                        content='foo',
                        options='bar',
                        message_type=Message.Type.MULTIPLE_CHOICE,
                        transitions='not a node'
                    ),
                    Node(
                        name='node2',
                        content='foo',
                        options='bar',
                        message_type=Message.Type.MULTIPLE_CHOICE,
                        transitions='exit'
                    ),
                ]
            )
        )

        # No exit
        self.assertRaises(
            ValueError,
            lambda: DirectedGraph(
                name="Foo",
                start_node="node1",
                nodes=[
                    Node(
                        name='node1',
                        content='foo',
                        options='bar',
                        message_type=Message.Type.MULTIPLE_CHOICE,
                        transitions='node2'
                    ),
                    Node(
                        name='node2',
                        content='foo',
                        options='bar',
                        message_type=Message.Type.MULTIPLE_CHOICE,
                        transitions='node1'
                    ),
                ]
            )
        )

        # Bad start node
        self.assertRaises(
            KeyError,
            lambda: DirectedGraph(
                name="Foo",
                start_node="node that doesn't exist",
                nodes=[
                    Node(
                        name='node1',
                        content='foo',
                        options='bar',
                        message_type=Message.Type.MULTIPLE_CHOICE,
                        transitions='node2'
                    ),
                    Node(
                        name='node2',
                        content='foo',
                        options='bar',
                        message_type=Message.Type.MULTIPLE_CHOICE,
                        transitions='exit'
                    ),
                ]
            )
        )
