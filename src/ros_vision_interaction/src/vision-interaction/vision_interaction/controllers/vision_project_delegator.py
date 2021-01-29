#!/usr/bin/env python3.8
import datetime
import logging

logging.basicConfig(level=logging.INFO)

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class VisionProjectDelegator:

    def __init__(
            self,
            statedb,
            paramdb,
            checkin_window_seconds=15,
            is_run_demo_interaction=False
    ):
        self._state_database = statedb
        self._param_database = paramdb
        self._checkin_window_seconds = datetime.timedelta(seconds=checkin_window_seconds)

        self._is_run_demo_interaction = is_run_demo_interaction
        self._is_run_interaction = False

        if self._is_run_demo_interaction:
            self._reset_database()

    def determine_interaction_type(self):
        logging.info("Determining interaction type")
        interaction_type = None
        current_time = datetime.datetime.now()
        last_interaction_time = self._state_database.get("last interaction time")

        if self._is_run_demo_interaction:
            if last_interaction_time is None:
                self._is_run_interaction = True
            else:
                self._is_run_interaction = \
                    current_time - last_interaction_time > \
                    datetime.timedelta(minutes=self._param_database["minutes between demo interactions"])
            if self._is_run_interaction:
                interaction_type = "demo interaction"
        else:
            if self._is_first_interaction():
                interaction_type = "first interaction"
            else:
                if self._is_time_for_scheduled_interaction():
                    interaction_type = "scheduled interaction"

        return interaction_type

    def _is_first_interaction(self):
        return not self._state_database.is_set("first interaction time")

    def _is_time_for_scheduled_interaction(self):
        current_time = datetime.datetime.now()
        start_time = self._state_database.get("next checkin time") - self._checkin_window_seconds
        end_time = self._state_database.get("next checkin time") + self._checkin_window_seconds
        return start_time < current_time < end_time

    def _reset_database(self):
        keys = self._state_database.get_keys()
        for key in keys:
            self._state_database.set(key, None)
