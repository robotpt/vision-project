import mock
import unittest

from interaction_engine.interface import Interface
from interaction_engine.message import Message

valid_message = Message(
    content="Hello there!",
    message_type=Message.Type.MULTIPLE_CHOICE,
    options=["Hello!", "Hi!"]
)


def test_input_function(message, data_dict):
    print("Input function called")


def test_output_function(message, data_dict):
    print("Output function called")


class TestState(unittest.TestCase):

    def test_init_with_valid_functions(self):
        interface = Interface(
            input_function=test_input_function,
            output_function=test_output_function
        )
        interface.run(valid_message, None)

    def test_init_with_invalid_functions(self):
        self.assertRaises(
            TypeError,
            Interface,
            input_function="Hello!",
            output_function="Hello!"
        )

    def test_run_interface(self):
        with mock.patch('__builtin__.input', return_value="Hello!") as mock_input_function:
            interface = Interface(
                input_function=input,
                output_function=test_output_function
            )
            interface.run("Hello!", None)
            mock_input_function.assert_called_with("Hello!", None)


if __name__ == '__main__':
    unittest.main()
