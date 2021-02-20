#!/usr/bin/env python3.8
import datetime
import logging

logging.basicConfig(level=logging.INFO)

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class VisionProjectDelegator:

    def __init__(
            self,
            statedb,
            checkin_window_seconds=15,
            is_run_demo_interaction=False
    ):
        self._state_database = statedb
        self._checkin_window_seconds = datetime.timedelta(seconds=checkin_window_seconds)

        self._is_run_demo_interaction = is_run_demo_interaction
        self._is_run_interaction = False

        # for testing purposes
        self._reset_database()

    def update(self):
        if not self._state_database.is_set("last update datetime"):
            self._state_database.set("last update datetime", datetime.datetime.now())
        if datetime.datetime.now().day > self._state_database.get("last update datetime").day:
            self._state_database.set("is done evaluation today", False)
        self._state_database.set("last update datetime", datetime.datetime.now())

    def determine_interaction_type(self):
        logging.info("Determining interaction type")

        if self._is_first_interaction():
            interaction_type = "first interaction"
        else:
            if self._is_time_for_scheduled_interaction():
                interaction_type = "scheduled interaction"
            elif self._is_off_checkin():
                interaction_type = "evaluation"
            elif self._is_run_prompted():
                interaction_type = "prompted interaction"
            else:
                interaction_type = None

        return interaction_type

    def _is_first_interaction(self):
        return not self._state_database.is_set("first interaction time")

    def _is_run_prompted(self):
        return self._state_database.get("is run prompted")

    def _is_time_for_scheduled_interaction(self):
        if self._is_off_checkin():
            self._state_database.set("is off checkin", False)
            return True
        else:
            if not self._state_database.get("next checkin datetime"):
                return False
            current_time = datetime.datetime.now()
            start_time = self._state_database.get("next checkin datetime") - self._checkin_window_seconds
            end_time = self._state_database.get("next checkin datetime") + self._checkin_window_seconds
            return start_time < current_time < end_time and not self._state_database.get("is done evaluation today")

    def _reset_database(self):
        keys = self._state_database.get_keys()
        for key in keys:
            self._state_database.set(key, None)

    def _is_off_checkin(self):
        return self._state_database.get("is off checkin")
