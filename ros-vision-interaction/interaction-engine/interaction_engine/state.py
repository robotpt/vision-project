#!/usr/bin/env python
import logging
from interaction_engine.message import Message

logging.basicConfig(level=logging.INFO)


def empty_function():
    pass


class State(object):

    def __init__(
            self,
            name,
            message_type,
            content,
            next_states,
            transitions=None,
            database_key_to_write=None,
            database_keys_to_read=None,
            args=None,
            check_input_function=None
    ):

        if type(name) is not str:
            raise TypeError("Name must be a string.")
        self._name = name

        self._check_valid_inputs(
            message_type,
            transitions,
            database_key_to_write,
            database_keys_to_read,
            args,
            check_input_function,
        )

        next_states = self._make_sure_next_states_match_type(message_type, next_states)
        self._next_states = []
        for next_state in next_states:
            self.add_next_state(next_state)

        self._transitions = self._make_sure_transitions_are_valid(message_type, transitions)

        self._message = self._create_message(
            content,
            database_keys_to_read,
            message_type,
            transitions,
            args
        )

        if database_key_to_write is not None:
            self._database_key_to_write = database_key_to_write
        else:
            self._database_key_to_write = self._name

        if check_input_function is not None:
            self._check_input_function = check_input_function
        else:
            self._check_input_function = empty_function

    def _check_valid_inputs(
            self,
            message_type,
            transitions,
            database_key_to_write,
            database_keys_to_read,
            args,
            check_input_function,
    ):
        valid_message_types = [
            Message.Type.MULTIPLE_CHOICE,
            Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
            Message.Type.TEXT_ENTRY,
            Message.Type.NO_INPUT,
            Message.Type.TIME_ENTRY,
        ]
        self._check_valid_message_type(message_type, valid_message_types)
        self._check_database_key_to_write(database_key_to_write)
        self._check_database_key_to_read(database_keys_to_read)
        self._check_args(args)
        self._check_functions(check_input_function)

    def _check_valid_message_type(self, message_type, valid_message_types):
        if message_type not in valid_message_types:
            raise TypeError("Message type must be one of the following: {}".format(valid_message_types))

    def _check_database_key_to_read(self, database_keys_to_read):
        if database_keys_to_read is not None:
            if type(database_keys_to_read) is not list:
                raise TypeError("Database keys to read must be a list")
            for key in database_keys_to_read:
                if type(key) is not str:
                    raise TypeError("Database keys to read must be strings")

    def _check_database_key_to_write(self, database_key_to_write):
        if database_key_to_write is not None:
            if type(database_key_to_write) is not str:
                raise TypeError("Database key to write must be a string")

    def _check_args(self, args):
        if args is not None:
            if len(args) != 2:
                raise IOError("2 args required: minute intervals and time to display")
            for arg in args:
                if type(arg) is not str:
                    raise TypeError("Args must be strings")

    def _check_functions(self, check_input_function):
        if check_input_function is not None:
            if not callable(check_input_function):
                raise IOError("Input check function must be callable")

    def _make_sure_next_states_match_type(self, message_type, next_states):
        if type(next_states) is not list:
            raise TypeError("Next states must be a list.")
        else:
            if message_type not in [Message.Type.MULTIPLE_CHOICE, Message.Type.MULTIPLE_CHOICE_ONE_COLUMN]:
                if len(next_states) > 1:
                    raise IOError("Cannot have more than one next state for non-multiple-choice messages.")
        return next_states

    def add_next_state(self, next_state):
        if type(next_state) is not str:
            raise TypeError("Next state must be a string.")
        if next_state in self._next_states:
            raise ValueError("{} is already a next state.".format(next_state))
        self._next_states.append(next_state)

    def _make_sure_transitions_are_valid(self, message_type, transitions):
        if message_type in [Message.Type.MULTIPLE_CHOICE, Message.Type.MULTIPLE_CHOICE_ONE_COLUMN]:
            if len(transitions) == 0:
                raise ValueError("Multiple choice messages must have transitions.")
            if type(transitions) is not dict:
                raise TypeError("Transitions must be a dict for multiple choice messages.")
            else:
                for option in list(transitions.keys()):
                    if type(option) is not str:
                        raise TypeError("Transition keys must be strings.")
                for next_state in list(transitions.values()):
                    if next_state not in self._next_states:
                        raise ValueError("Transition values must be valid next states.")
        return self._make_transitions(message_type, transitions)

    def _make_transitions(self, message_type, transitions):
        if message_type in [Message.Type.MULTIPLE_CHOICE, Message.Type.MULTIPLE_CHOICE_ONE_COLUMN]:
            next_state_names = list(transitions.values())
            new_transitions = {i: next_state_names[i] for i in range(len(next_state_names))}
        else:
            if len(self._next_states) > 0:
                new_transitions = {0: self._next_states[0]}
            else:
                new_transitions = None
        return new_transitions

    def _create_message(self, content, database_keys_to_read, message_type, transitions, args):
        if message_type in [Message.Type.TEXT_ENTRY, Message.Type.NO_INPUT, Message.Type.TIME_ENTRY]:
            options = ["Next"]
        else:
            options = list(transitions.keys())

        return Message(
            content=content,
            message_type=message_type,
            keys_to_read=database_keys_to_read,
            options=options,
            args=args
        )

    def __repr__(self):
        return "State: {}, {}, Transitions: {}".format(self._name, self._message, self._transitions)

    @property
    def name(self):
        return self._name

    @property
    def next_states(self):
        return self._next_states

    @property
    def transitions(self):
        return self._transitions

    @property
    def message(self):
        return self._message

    @property
    def database_key_to_write(self):
        return self._database_key_to_write

    @property
    def check_input_function(self):
        return self._check_input_function

    def get_next_state(self, user_input):
        if len(self._next_states) == 1:
            return self._next_states[0]
        else:
            return self._transitions[user_input]
