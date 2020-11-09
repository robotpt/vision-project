#!/usr/bin/env python
import logging
import math
from interaction_engine.interface import Interface
from interaction_engine.message import Message

logging.basicConfig(level=logging.INFO)


class TerminalInterface(Interface):

    def __init__(self):
        super(TerminalInterface, self).__init__(
            input_function=self.get_user_input,
            output_function=self.print_message
        )

    def run(self, message, data_dict):
        return super(TerminalInterface, self).run(
            message,
            data_dict
        )

    def print_message(self, message, data_dict):
        print(self.format_message(message, data_dict))

    def format_message(self, message, data_dict):
        if type(message) is not Message:
            raise TypeError("Invalid message class.")

        result = self.update_message(message, data_dict) + "\n"
        if message.message_type is Message.Type.MULTIPLE_CHOICE:
            for i in range(len(message.options)):
                result += "{option_num}: {text}".format(option_num=i, text=message.options[i]) + "\n"
        if message.message_type is not Message.Type.NO_INPUT:
            result += "\n>> "
        return result

    def get_user_input(self, message, data_dict):
        if message.message_type is Message.Type.MULTIPLE_CHOICE:
            return self.get_multiple_choice_input(message)
        elif message.message_type is Message.Type.TEXT_ENTRY:
            return self.get_text_input(message)
        else:
            return ""

    def get_multiple_choice_input(self, message):
        user_input = input()
        # if not user_input.isdigit():
        #     print("Not a valid response.")
        #     user_input = self.get_multiple_choice_input(message)
        user_input = int(user_input)
        if not self.is_valid_input(message, user_input):
            print("Not a valid option.")
            user_input = self.get_multiple_choice_input(message)
        return user_input

    def get_text_input(self, message):
        user_input = raw_input()
        if not self.is_valid_input(message, user_input):
            print("Not a valid response.")
            user_input = self.get_text_input(message)
        return user_input

    def is_valid_input(self, message, user_input):
        if message.message_type is Message.Type.MULTIPLE_CHOICE:
            valid_inputs = [i for i in range(len(message.options))]
            return user_input in valid_inputs
        else:
            return type(user_input) is str

