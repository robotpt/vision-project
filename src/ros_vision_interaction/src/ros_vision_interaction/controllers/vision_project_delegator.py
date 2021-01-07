#!/usr/bin/env python3.8
import datetime
import logging
import pprint
import schedule


logging.basicConfig(level=logging.INFO)

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class VisionProjectDelegator:

    def __init__(
            self,
            mongodb_statedb,
            is_clear_state=False
    ):
        self._state_database = mongodb_statedb
        self._set_initial_db_keys()

        self._is_start_interaction = False

    def get_interaction_type(self):
        return None

    def _set_initial_db_keys(self):
        keys = [
            "user_name",
            "time_for_next_interaction"
        ]
        for key in keys:
            try:
                self._state_database.create(key, None)
            except KeyError:
                logging.info("{} already exists".format(key))
