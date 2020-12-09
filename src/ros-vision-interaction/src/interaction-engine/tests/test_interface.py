from interaction_engine.interfaces.client_and_server_interface import ClientAndServerInterface
from interaction_engine.messager import Message
from interaction_engine.json_database import Database
from robotpt_common_utils import lists

import unittest
import os

db_file = 'test_db.json'


class TestInterface(unittest.TestCase):

    def setUp(self) -> None:

        db = Database(db_file)
        db["user_name"] = None
        db["question_idx"] = 0
        db["answers"] = None

        self._db = db
        self._io = TestIO()
        self._interface = ClientAndServerInterface(
            *[self._io.output_fn, self._io.input_fn]*2,
            database=self._db
        )

    def tearDown(self) -> None:

        os.remove(db_file)

    def test_single_choice(self):

        content = 'Question'
        options = 'choice'
        choice = options
        message = Message(
            content=content,
            options=options,
            message_type=Message.Type.MULTIPLE_CHOICE,
        )
        self._io.set_input(choice)
        self.assertIsNone(
            self._io.get_output()
        )
        self.assertEqual(
            choice,
            self._interface.run(message)
        )
        self.assertEqual(
            content,
            self._io.get_output()
        )

    def test_multiple_choice(self):

        content = 'Question'
        options = ['choice', 'not choice']
        choice = options[0]
        message = Message(
            content=content,
            options=options,
            message_type=Message.Type.MULTIPLE_CHOICE,
        )
        self._io.set_input(choice)
        self.assertIsNone(
            self._io.get_output()
        )
        self.assertEqual(
            choice,
            self._interface.run(message)
        )
        self.assertEqual(
            content,
            self._io.get_output()
        )

    def test_error_message(self):

        content = 'Question'
        options = ['choice', 'not choice']
        error_message = 'Error'
        error_options = 'Oops'
        choice = options[0]
        message = Message(
            content=content,
            options=options,
            message_type=Message.Type.MULTIPLE_CHOICE,
            error_message=error_message,
            error_options=error_options,
            tests=lambda x: x in options,
        )
        self._io.set_input(
            [
                "invalid option",
                error_options,
                "invalid option",
                error_options,
                choice
             ]
        )
        self._interface.run(message)
        out = self._io.get_output()
        truth_out = [content, error_message, content, error_message, content]
        for i in range(len(truth_out)):
            self.assertEqual(
                truth_out[i],
                out[i]
            )

    def test_confirm(self):

        content = 'Question'
        options = ['choice', 'not choice']
        choice = options[0]
        message = Message(
            content=content,
            options=options,
            message_type=Message.Type.MULTIPLE_CHOICE,
            is_confirm=True,
        )
        self._io.set_input(
            [
                choice,
                'Yes'
            ]
        )
        self._interface.run(message)
        out = self._io.get_output()
        truth_out = [content, f"'{str(choice)}', right?"]
        for i in range(len(truth_out)):
            self.assertEqual(
                truth_out[i],
                out[i]
            )

    def test_write_db_entry(self):

        content = 'Question'
        options = 'choice'
        choice = options

        db_key = 'out'
        message = Message(
            content=content,
            options=options,
            message_type=Message.Type.MULTIPLE_CHOICE,
            result_db_key=db_key,
        )
        self._io.set_input(choice)
        self.assertRaises(
            KeyError,
            self._interface.run,
            message,
        )

        self._db.create_key(db_key)
        self.assertTrue(
            db_key in self._db.get_keys()
        )
        self.assertFalse(
            self._db.is_set(db_key)
        )

        self._io.set_input(choice)
        self._interface.run(message)
        self.assertTrue(
            db_key in self._db.get_keys()
        )
        self.assertEqual(
            choice,
            self._db[db_key]
        )

    def test_append_db_entry(self):

        content = 'Question'
        options = ['choice', 'not choice']

        choice1 = options[1]
        choice2 = options[0]
        choices = [choice1, choice2]

        db_key = 'out'
        self._db.create_key(db_key)
        self.assertTrue(
            db_key in self._db.get_keys()
        )
        self.assertFalse(
            self._db.is_set(db_key)
        )

        message = Message(
            content=content,
            options=options,
            message_type=Message.Type.MULTIPLE_CHOICE,
            result_db_key=db_key,
        )
        self._io.set_input(choices)
        self._interface.run(message)
        self._interface.run(message)

        self.assertTrue(
            db_key in self._db.get_keys()
        )

        truth_out = choices
        out = self._db.create_key(db_key)
        for i in range(len(truth_out)):
            self.assertEqual(
                truth_out[i],
                out[i]
            )

    def test_create_new_db_entry(self):

        content = 'Question'
        options = 'choice'
        choice = options

        db_key = 'out'
        self.assertFalse(
            db_key in self._db.get_keys()
        )

        message = Message(
            content=content,
            options=options,
            message_type=Message.Type.MULTIPLE_CHOICE,
            result_db_key=db_key,
        )
        self._io.set_input(choice)
        self._interface._is_create_db_key_if_not_exist = True
        self._interface.run(message)

        self.assertTrue(
            db_key in self._db.get_keys()
        )
        self.assertEqual(
            choice,
            self._db[db_key]
        )

class TestIO:

    def __init__(self):
        self._output = []
        self._response = []

    def output_fn(self, message):
        self._output.append(message.content)

    def input_fn(self, _):
        return self._response.pop(0)

    def set_input(self, values):
        self._response = lists.make_sure_is_iterable(values)

    def get_output(self):
        if len(self._output) < 1:
            return None
        elif len(self._output) == 1:
            return self._output[0]
        else:
            return self._output

