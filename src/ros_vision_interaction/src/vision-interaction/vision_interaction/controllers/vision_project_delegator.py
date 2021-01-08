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

        self._is_start_interaction = False

    def determine_interaction_type(self):
        if self._is_first_interaction():
            return "first interaction"

    def _is_first_interaction(self):
        return not self._state_database.is_set("first interaction time")
