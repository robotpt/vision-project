import random
import unittest

from interaction_engine.message import Message
from interaction_engine.state import State
from interaction_engine.state_collection import StateCollection

valid_name = "state collection 1"
valid_init_state_name = "state 1"
valid_state_1 = State(
    name="state 1",
    message_type=Message.Type.MULTIPLE_CHOICE,
    content="Hello!",
    next_states=["state 2"],
    transitions={"Hello!": "state 2", "Hi!": "state 2"}
)
valid_state_2 = State(
    name="state 2",
    message_type=Message.Type.NO_INPUT,
    content="Goodbye!",
    next_states=["exit"],
    transitions={"Bye!": "exit"}
)


class TestStateCollection(unittest.TestCase):

    def test_init_with_valid_names(self):
        valid_names = [
            "state collection 1",
            "state collection 2",
            "state collection 3",
        ]
        for valid_name in valid_names:
            state_collection = StateCollection(
                name=valid_name,
                init_state_name=valid_init_state_name,
                states=[valid_state_1, valid_state_2],
            )
            self.assertEqual(valid_name, state_collection.name)

    def test_init_with_invalid_names(self):
        invalid_names = [
            123,
            ["a", "b", "c"]
        ]
        for invalid_name in invalid_names:
            self.assertRaises(
                TypeError,
                StateCollection,
                name=invalid_name
            )

    def test_transition(self):
        state_collection = StateCollection(
            name=valid_name,
            init_state_name=valid_init_state_name,
            states=[valid_state_1, valid_state_2],
        )
        self.assertEqual(state_collection.current_state, "state 1")
        state_collection.transition(0)
        self.assertEqual(state_collection.current_state, "state 2")
        state_collection.transition(0)
        self.assertEqual(state_collection.current_state, "exit")

    def test_add_state(self):
        state1 = State(
            name="state 1",
            message_type=Message.Type.MULTIPLE_CHOICE,
            content="Hello!",
            next_states=["state 2", "state 3"],
            transitions={"Hello!": "state 2", "Hi!": "state 3"}
        )

        state2 = State(
            name="state 2",
            message_type=Message.Type.MULTIPLE_CHOICE,
            content="Hello again!",
            next_states=["state 1"],
            transitions={"Hello!": "state 1", "Hi!": "state 1"}
        )

        state_collection = StateCollection(
            name=valid_name,
            init_state_name=valid_init_state_name,
            states=[state1, state2],
        )

        state3 = State(
            name="state 3",
            message_type=Message.Type.MULTIPLE_CHOICE,
            content="This is a new state.",
            next_states=["state 1", "state 2", "exit"],
            transitions={"1": "state 1", "2": "state 2", "3": "exit"}
        )

        state_collection.add_state(state3)

        self.assertIn("state 3", state_collection.states.keys())

        while state_collection.is_running:
            random_choice = random.choice(
                range(len(state_collection.states[state_collection.current_state].next_states
                          )))
            state_collection.transition(random_choice)

    def test_state_collection_doesnt_run_with_invalid_next_states(self):
        state1 = State(
            name="state 1",
            message_type=Message.Type.MULTIPLE_CHOICE,
            content="Hello!",
            next_states=["state 2", "state 3"],
            transitions={"Hello!": "state 2", "Hi!": "state 3"}
        )

        state2 = State(
            name="state 2",
            message_type=Message.Type.MULTIPLE_CHOICE,
            content="Hello again!",
            next_states=["state 1"],
            transitions={"Hello!": "state 1", "Hi!": "state 1"}
        )

        state_collection = StateCollection(
            name=valid_name,
            init_state_name=valid_init_state_name,
            states=[state1, state2],
        )

        state3 = State(
            name="state 3",
            message_type=Message.Type.MULTIPLE_CHOICE,
            content="This is a new state.",
            next_states=["state 1", "state 2", "exit"],
            transitions={"1": "state 1", "2": "state 2", "3": "exit"}
        )

        self.assertRaises(
            ValueError,
            state_collection.transition,
            0
        )

    def test_build_states_from_dict(self):
        states_dict = {
            "hello there": {
                "content": ["Hello there!"],
                "message type": "multiple choice one column",
                "next states": ["what's your name"],
                "transitions": {"Hello": "what's your name"}
            },
            "what's your name": {
                "content": ["What's your name?"],
                "message type": "text entry",
                "next states": ["exit"],
                "transitions": {"Next": "exit"}
            },
        }
        state_collection = StateCollection(
            name="test state collection with dict",
            init_state_name="hello there",
            states=states_dict
        )
        expected_states = [
            State(
                name="hello there",
                content=["Hello there!"],
                message_type="multiple choice one column",
                next_states=["what's your name"],
                transitions={"Hello": "what's your name"}
            ),
            State(
                name="what's your name",
                content=["What's your name?"],
                message_type="text entry",
                next_states=["exit"],
                transitions={"Next": "exit"}
            )
        ]
        for expected_state in expected_states:
            actual_state = state_collection.states[expected_state.name]
            self.assertEqual(expected_state.name, actual_state.name)
            self.assertEqual(expected_state.next_states, actual_state.next_states)
            self.assertEqual(expected_state.transitions, actual_state.transitions)
            self.assertEqual(expected_state.message.content, actual_state.message.content)
            self.assertEqual(expected_state.message.message_type, actual_state.message.message_type)


if __name__ == '__main__':
    unittest.main()
