import unittest

from interaction_engine.message import Message
from interaction_engine.state import State
from interaction_engine.utils import make_sure_is_list

valid_name = "test state"
valid_message_type = Message.Type.TEXT_ENTRY
valid_content = "Hello there!"
valid_next_states = ["test state 2"]
exit_transition = {"Next": "exit"}


class TestState(unittest.TestCase):
    def test_check_name(self):
        valid_names = [
            "state 1",
            "state 2",
            "state 3"
        ]
        for valid_name in valid_names:
            state = State(
                name=valid_name,
                message_type=valid_message_type,
                content=valid_content,
                next_states=valid_next_states,
            )
            self.assertEqual(valid_name, state.name)
        invalid_names = [
            1,
            [1, 2, 3],
            {1: "a", 2: "b"}
        ]
        for invalid_name in invalid_names:
            self.assertRaises(
                TypeError,
                State,
                name=invalid_name
            )

    def test_create_multiple_choice_message(self):
        valid_message_types = [
            Message.Type.MULTIPLE_CHOICE,
            Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
        ]
        valid_contents = "Hello there!"
        valid_next_state = ["test state 2"]
        valid_transitions = [
            {"Hi!": "test state 2"},
            {"Hi!": "test state 2"},
        ]
        expected_options = [
            ["Hi!"],
            ["Hi!"],
        ]
        for i in range(2):
            state = State(
                name=valid_name,
                message_type=valid_message_types[i],
                content=valid_contents,
                next_states=valid_next_state,
                transitions=valid_transitions[i]
            )
            self.assertEqual(make_sure_is_list(valid_contents), state.message.content)
            self.assertEqual(valid_message_types[i], state.message.message_type)
            self.assertEqual(expected_options[i], state.message.options)

    def test_create_text_entry_message(self):
        valid_message_type = Message.Type.TEXT_ENTRY
        valid_contents = "Hello there!"
        valid_next_state = ["test state 2"]
        valid_transition = None
        expected_options = ["Next"]

        state = State(
            name=valid_name,
            message_type=valid_message_type,
            content=valid_contents,
            next_states=valid_next_state,
            transitions=valid_transition
        )
        self.assertEqual(make_sure_is_list(valid_contents), state.message.content)
        self.assertEqual(valid_message_type, state.message.message_type)
        self.assertEqual(expected_options, state.message.options)

    def test_create_no_input_message(self):
        valid_message_type = Message.Type.NO_INPUT
        valid_contents = "Hello there!"
        valid_next_state = ["test state 2"]
        valid_transition = None
        expected_options = ["Next"]

        state = State(
            name=valid_name,
            message_type=valid_message_type,
            content=valid_contents,
            next_states=valid_next_state,
            transitions=valid_transition
        )
        self.assertEqual(make_sure_is_list(valid_contents), state.message.content)
        self.assertEqual(valid_message_type, state.message.message_type)
        self.assertEqual(expected_options, state.message.options)

    def test_create_time_entry_message(self):
        valid_message_type = Message.Type.TIME_ENTRY
        valid_contents = "Hello there!"
        valid_next_state = ["test state 2"]
        valid_transition = None
        valid_args = [
            ["5", "12:00"],
            ["10", "10:30"]
        ]
        expected_options = ["Next"]

        for i in range(2):
            state = State(
                name=valid_name,
                message_type=valid_message_type,
                content=valid_contents,
                next_states=valid_next_state,
                transitions=valid_transition,
                args=valid_args[i]
            )
            self.assertEqual(make_sure_is_list(valid_contents), state.message.content)
            self.assertEqual(valid_message_type, state.message.message_type)
            self.assertEqual(expected_options, state.message.options)
            self.assertEqual(valid_args[i], state.message.args)

    def test_get_next_state(self):
        state1 = State(
            name="state 1",
            message_type=Message.Type.MULTIPLE_CHOICE,
            content="Hello there!",
            next_states=["state 2", "state 3", "state 4"],
            transitions={"a": "state 2", "b": "state 3", "c": "state 4"},
        )

        self.assertEqual("state 2", state1.get_next_state(state1.transitions.values().index("state 2")))
        self.assertEqual("state 3", state1.get_next_state(state1.transitions.values().index("state 3")))
        self.assertEqual("state 4", state1.get_next_state(state1.transitions.values().index("state 4")))

    def test_pre_and_post_functions(self):
        def check_input():
            return 0

        state1 = State(
            name="state 1",
            message_type=Message.Type.NO_INPUT,
            content="Hello there!",
            next_states=["exit"],
            transitions=exit_transition,
            check_input_function=check_input
        )

        self.assertEqual(0, state1.check_input_function())

        state1 = State(
            name="state 1",
            message_type=Message.Type.NO_INPUT,
            content="Hello there!",
            next_states=["exit"],
            transitions=exit_transition,
        )

        self.assertEqual(None, state1.check_input_function())

        invalid_functions = [
            0,
            "Hello",
            [1, 2, 3],
        ]

        for invalid_check_input_function in invalid_functions:
            self.assertRaises(
                IOError,
                State,
                name=valid_name,
                message_type=valid_message_type,
                content=valid_content,
                next_states=valid_next_states,
                check_input_function=invalid_check_input_function
            )

    def test_database_key_to_write(self):
        state1 = State(
            name="state 1",
            message_type=Message.Type.NO_INPUT,
            content="Hello there!",
            next_states=["exit"],
            transitions=exit_transition,
            database_key_to_write="hello"
        )
        state2 = State(
            name="state 2",
            message_type=Message.Type.NO_INPUT,
            content="Hello there!",
            next_states=["exit"],
            transitions=exit_transition,
        )
        self.assertEqual("hello", state1.database_key_to_write)
        self.assertEqual("state 2", state2.database_key_to_write)

        invalid_keys_to_write = [
            123,
            True,
            ["hello"],
        ]
        for invalid_key in invalid_keys_to_write:
            self.assertRaises(
                TypeError,
                State,
                name="state 1",
                message_type=Message.Type.NO_INPUT,
                content="Hello!",
                next_states=["exit"],
                transitions=exit_transition,
                database_key_to_write=invalid_key
            )


if __name__ == '__main__':
    unittest.main()
