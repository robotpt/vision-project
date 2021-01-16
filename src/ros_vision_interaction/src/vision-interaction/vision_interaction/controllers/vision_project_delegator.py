#!/usr/bin/env python3.8
import datetime
import logging


logging.basicConfig(level=logging.INFO)

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"
TIME_BETWEEN_DEMO_INTERACTIONS = datetime.timedelta(minutes=5)
TIME_WINDOW_FOR_CHECKIN = datetime.timedelta(seconds=30)


class VisionProjectDelegator:

    def __init__(
            self,
            mongodb_statedb,
            is_run_demo_interaction=False
    ):
        self._state_database = mongodb_statedb

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
                self._is_run_interaction = current_time - last_interaction_time > TIME_BETWEEN_DEMO_INTERACTIONS
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
        start_time = self._state_database.get("next checkin time") - TIME_WINDOW_FOR_CHECKIN
        end_time = self._state_database.get("next checkin time") + TIME_WINDOW_FOR_CHECKIN
        return start_time < current_time < end_time

    def _reset_database(self):
        keys = self._state_database.get_keys()
        for key in keys:
            self._state_database.set(key, None)
