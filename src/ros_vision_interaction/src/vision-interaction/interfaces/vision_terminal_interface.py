#!/usr/bin/env python3.8
import textwrap

from interaction_engine.interfaces import Interface
from interaction_engine.messager import Message
from robotpt_common_utils import math_tools, lists


class VisionTerminalInterface(Interface):

    def __init__(
            self,
            state_database,
            seconds_until_timeout=None,
            is_create_db_key_if_not_exist=True,
    ):
        self._state_database = state_database
        self.seconds_until_timeout = seconds_until_timeout
        super().__init__(
            input_fn=self.get_input,
            output_fn=self.print_message,
            database=self._state_database,
            is_create_db_key_if_not_exist=is_create_db_key_if_not_exist
        )

    def print_message(self, message, width=50):
        if type(message) is not Message:
            raise TypeError("Invalid message class.")

        print("=====================")
        wrapper = textwrap.TextWrapper(width=width)
        text_list = wrapper.wrap(text=message.content)

        for element in text_list:
            print(element)

        if message.message_type == Message.Type.MULTIPLE_CHOICE or message.message_type == "multiple choice one column":
            self.print_multiple_choice(message.options)
        elif message.message_type == Message.Type.DIRECT_INPUT or message.message_type == Message.Type.TIME_ENTRY:
            self.print_text_entry(message)
        else:
            self.print_gui_entry(message)

    def print_multiple_choice(self, options):
        self._print_enumerated_list(options)

    def _print_enumerated_list(self, options):
        options = lists.make_sure_is_iterable(options)
        for i in range(len(options)):
            print(f" {i}. " + options[i])

    def print_text_entry(self, message):
        if message.message_type == Message.Type.TIME_ENTRY:
            print("Note: time must be in the form: 'HH:MM AM/PM'")

    def print_gui_entry(self, message):
        print(f"Note: message type {message.message_type} cannot be displayed on command line.")

    def get_input(self, message):
        if message.message_type == Message.Type.MULTIPLE_CHOICE or message.message_type == "multiple choice one column":
            return self.input_multiple_choice(message.options)
        elif message.message_type == Message.Type.DIRECT_INPUT or message.message_type == Message.Type.TIME_ENTRY:
            return self.input_text_entry(message)
        else:
            return self.input_gui_entry(message)

    def input_multiple_choice(self, options):
        response = None
        while response not in range(len(options)):
            response_str = input(">>> ")
            if math_tools.is_int(response_str):
                response = int(response_str)

        return options[response]

    def input_text_entry(self, _):
        response = ""
        while len(response) == 0:
            response = input(">>> ")
        print(f"Text entry response was: {response}")
        return response

    def input_gui_entry(self, _):
        options = ["Next"]
        self.input_multiple_choice(options)

    @staticmethod
    def _message_fn_from_dict(msg_dict):
        def _fn(msg: Message):
            return msg_dict[msg.message_type](msg)
        return _fn
