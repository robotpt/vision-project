#!/usr/bin/env python
import logging

from interaction_engine.database import Database
from interaction_engine.interface import Interface
from interaction_engine.state_collection import StateCollection

logging.basicConfig(level=logging.INFO)


# TODO: check that database keys to read and write correspond to actual database keys for each state
class InteractionEngine(object):

    def __init__(
            self,
            state_collection,
            database_manager,
            interface,
    ):

        if type(state_collection) is not StateCollection:
            raise TypeError("Not a valid state collection.")
        self._state_collection = state_collection

        if type(database_manager) is not Database:
            raise TypeError("Invalid database manager class.")
        self._database_manager = database_manager

        if not issubclass(interface.__class__, Interface):
            raise TypeError("Invalid interface class.")
        self._interface = interface

        self._is_run_next_state = True

    @property
    def state_collection(self):
        return self._state_collection

    @property
    def database_manager(self):
        return self._database_manager

    def get_data_from_db_manager(self, keys):
        if keys is not None:
            return {key: self._database_manager[key] for key in keys}
        else:
            return None

    def run(self):
        self._state_collection.is_running = True
        states_dict = self._state_collection.states

        while self._is_run_next_state:

            current_state = states_dict[self._state_collection.current_state]
            current_message = current_state.message

            user_input = self.format_message_and_get_input(current_message)

            self.format_and_save_input(current_message, current_state, user_input)

            self._state_collection.transition(user_input)

            if not self._state_collection.is_running:
                self._state_collection.reset()
                self._is_run_next_state = False

    def format_message_and_get_input(self, current_message):
        data_for_message = self.get_data_from_db_manager(current_message.keys_to_read)
        user_input = self._interface.run(current_message, data_for_message)
        return user_input

    def format_and_save_input(self, current_message, current_state, user_input):
        data = self._interface.format_user_input_for_database(current_message, user_input)
        self._database_manager[current_state.database_key_to_write] = data
