import unittest

from interaction_engine.message import Message
from interaction_engine.utils import make_sure_is_list

valid_content = ["Hello there!"]
valid_type = Message.Type.TEXT_ENTRY
valid_options = ["Hi!", "Hello!"]


class TestMessage(unittest.TestCase):
    def test_check_contents(self):
        valid_contents = [
            "Hello there!",
            ["Hello!", "Hi there!"],
            ["How are you doing today?"]
        ]
        for valid_content in valid_contents:
            message = Message(
                content=valid_content,
                message_type=valid_type,
            )
            self.assertEqual(make_sure_is_list(valid_content), message.content)

        invalid_contents = [
            1,
            [1, 2, 3],
            {1: "Hi", 2: "Hello"}
        ]
        for invalid_content in invalid_contents:
            self.assertRaises(
                TypeError,
                Message,
                content=invalid_content,
                message_type=valid_type
            )

    def test_check_multiple_choice_type(self):
        valid_types = [
            Message.Type.MULTIPLE_CHOICE,
            Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
        ]
        for valid_type in valid_types:
            message = Message(
                content=valid_content,
                message_type=valid_type,
                options=valid_options,
            )
            self.assertEqual(valid_type, message.message_type)

    def test_text_entry_type(self):
        valid_type = Message.Type.TEXT_ENTRY
        message = Message(
            content=valid_content,
            message_type=valid_type,
        )
        self.assertEqual(valid_type, message.message_type)

    def test_no_input_type(self):
        valid_type = Message.Type.NO_INPUT
        message = Message(
            content=valid_content,
            message_type=valid_type,
        )
        self.assertEqual(valid_type, message.message_type)

    def test_time_entry_type(self):
        valid_type = Message.Type.TIME_ENTRY
        message = Message(
            content=valid_content,
            message_type=valid_type,
            args=["5", "12:00"]
        )

    def test_check_options_correspond_to_type(self):
        multiple_choice_options_list = [
            ["Yes", "No"],
            ["Hello!", "Hi!", "Hi there!"]
        ]
        for multiple_choice_options in multiple_choice_options_list:
            message = Message(
                content=valid_content,
                message_type=Message.Type.MULTIPLE_CHOICE,
                options=multiple_choice_options
            )
            self.assertEqual(multiple_choice_options, message.options)
            message = Message(
                content=valid_content,
                message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
                options=multiple_choice_options
            )
            self.assertEqual(multiple_choice_options, message.options)
        self.assertRaises(
            IOError,
            Message,
            content=valid_content,
            message_type=Message.Type.MULTIPLE_CHOICE
        )
        self.assertRaises(
            IOError,
            Message,
            content=valid_content,
            message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN
        )

        message_types = [
            Message.Type.NO_INPUT,
            Message.Type.TEXT_ENTRY
        ]
        message = Message(
            content=valid_content,
            message_type=Message.Type.NO_INPUT,
        )
        self.assertEqual(["Next"], message.options)
        message = Message(
            content=valid_content,
            message_type=Message.Type.TEXT_ENTRY,
        )
        self.assertEqual(['Next'], message.options)

        for message_type in message_types:
            self.assertRaises(
                IOError,
                Message,
                content=valid_content,
                message_type=message_type,
                options=valid_options
            )


if __name__ == '__main__':
    unittest.main()
