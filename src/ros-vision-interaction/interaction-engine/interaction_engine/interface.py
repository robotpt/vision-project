#!/usr/bin/env python
import logging
import math

from interaction_engine.message import Message

logging.basicConfig(level=logging.INFO)


class Interface(object):

    def __init__(self, input_function, output_function=None):
        self._check_functions(input_function, output_function)
        self._output_function = output_function
        self._input_function = input_function

    def _check_functions(self, get_input_function, output_function):
        if output_function is not None:
            if not callable(output_function):
                raise TypeError("Output function must be callable.")
        if not callable(get_input_function):
            raise TypeError("Input function must be callable.")

    def run(self, message, data_dict):
        if self._output_function is not None:
            self._output_function(message, data_dict)
        return self._input_function(message, data_dict)

    def update_message(self, message, data_dict):
        if type(message) is not Message:
            raise TypeError("Not a valid message.")

        if data_dict:
            try:
                return message.get_random_content().format(**data_dict)
            except KeyError:
                logging.error("{} do not correspond to valid database keys.".format(message.keys_to_read))
                return message.get_random_content()
        else:
            return message.get_random_content()

    def format_user_input_for_database(self, message, user_input):
        if message.message_type in [Message.Type.MULTIPLE_CHOICE, Message.Type.MULTIPLE_CHOICE_ONE_COLUMN]:
            data = "{option_text}".format(
                option_text=self.get_option(message, user_input),
            )
        else:
            data = user_input
        return data

    def get_option(self, message, index):
        if type(message) is not Message:
            raise TypeError("Invalid message class.")

        max_index = len(message.options) - 1
        if type(index) not in (int, float):
            raise TypeError("Index must be an int or float")
        if index != math.floor(index):
            raise TypeError("Index must be an integer")
        if not 0 <= index < len(message.options):
            raise ValueError("Index must be 0 - {}.".format(max_index))
        return message.options[index]
