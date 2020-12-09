import unittest
from interaction_engine.messager.node import Node
from interaction_engine.messager import Message


class TestNode(unittest.TestCase):

    def test_multiple_choice_transition(self):
        options = ['Good', 'Okay', 'Bad']
        transitions = ['great', 'good', 'too bad']
        n = Node(
            name='name',
            content='How are you?',
            options=options,
            message_type='multiple choice',
            transitions=transitions
        )
        self.assertEqual(
            len(transitions),
            len(options)
        )
        for i in range(len(options)):
            self.assertEqual(
                transitions[i],
                n.get_transition(options[i])

            )

    def test_single_choice_transitions(self):
        options = ['Good', 'Okay', 'Bad']
        transition = 'oh'
        n = Node(
            name='name',
            content='How are you?',
            options=options,
            message_type='multiple choice',
            transitions=transition
        )
        for i in range(len(options)):
            self.assertEqual(
                transition,
                n.get_transition(options[i])
            )

    def test_fn_transition(self):

        transitions = ['not good', 'okay', 'good']

        def transition_fn(input):
            if input < 80:
                return 0
            elif input < 90:
                return 1
            elif input >= 90:
                return 2
            else:
                raise ValueError

        n = Node(
            name='check grade',
            content='How did you do?',
            options='is how I did',
            message_type='direct input',
            transitions=transitions,
            transition_fn=transition_fn,
        )
        for i in range(80):
            self.assertEqual(
                transitions[0],
                n.get_transition(i)
            )
        for i in range(80, 90):
            self.assertEqual(
                transitions[1],
                n.get_transition(i)
            )
        for i in range(90, 110):
            self.assertEqual(
                transitions[2],
                n.get_transition(i)
            )

    def test_valid_and_invalid_setups(self):
        one_option = 'o1'
        two_options = ['o1', 'o2']
        three_options = ['o1', 'o2', 'o3']

        multi = Message.Type.MULTIPLE_CHOICE
        direct = Message.Type.DIRECT_INPUT

        one_transition = 't1'
        two_transitions = ['t1', 't2']
        three_transitions = ['t1', 't2', 't3']

        is_fn = print
        none = None

        valid_sets = [
            [one_option, multi, one_transition, none],
            [two_options, multi, two_transitions, none],
            [three_options, multi, three_transitions, none],
            [three_options, multi, one_transition, none],
            [one_option, direct, one_transition, none],
            [one_option, direct, one_transition, is_fn],
            [one_option, direct, two_transitions, is_fn],
            [one_option, direct, three_transitions, is_fn],
        ]
        invalid_sets = [
            [one_option, multi, one_transition, is_fn],
            [two_options, multi, one_transition, is_fn],
            [three_options, multi, one_transition, is_fn],
            [one_transition, direct, two_transitions, none],
            [one_transition, direct, three_transitions, none],
            [two_options, direct, one_transition, is_fn],
            [three_options, direct, one_transition, is_fn],
        ]

        for set_ in valid_sets:
            options, message_type, transitions, transition_fn = set_
            node = Node(
                'name',
                content='Here is a question, or is it?',
                options=options,
                message_type=message_type,
                transitions=transitions,
                transition_fn=transition_fn
            )
            self.assertEqual(
                Node,
                type(node)
            )

        for set_ in invalid_sets:
            options, message_type, transitions, transition_fn = set_
            self.assertRaises(
                ValueError,
                Node,
                'name',
                content='Here is a question, or is it?',
                options=options,
                message_type=message_type,
                transitions=transitions,
                transition_fn=transition_fn
            )
