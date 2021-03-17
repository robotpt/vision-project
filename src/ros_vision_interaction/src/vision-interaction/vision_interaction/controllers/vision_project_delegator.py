#!/usr/bin/env python3.8
import datetime
import logging

from controllers.interaction_manager import Interactions

logging.basicConfig(level=logging.INFO)

START_INTERACTION_ACTION_NAME = "vision_project/start_interaction"


class VisionProjectDelegator:

    def __init__(
            self,
            statedb,
            update_window_seconds=15,
            scheduled_window_minutes=15,
            minutes_between_interactions=1,
    ):
        self._state_database = statedb
        self._update_window_seconds = datetime.timedelta(seconds=update_window_seconds)
        self._scheduled_window_minutes = datetime.timedelta(minutes=scheduled_window_minutes)
        self._minutes_between_interactions = datetime.timedelta(minutes=minutes_between_interactions)

        self._is_run_interaction = False

        # for testing purposes
        self._reset_database()

    def update(self):
        if not self._state_database.is_set("last update datetime"):
            self._state_database.set("last update datetime", datetime.datetime.now())
        if self._is_new_day():
            if not self._state_database.get("is done eval today"):
                num_of_days_since_last_eval = self._state_database.get("num of days since last eval")
                self._state_database.set("num of days since last eval", num_of_days_since_last_eval + 1)
            else:
                self._state_database.set("is done eval today", False)
        self._state_database.set("last update datetime", datetime.datetime.now())

    def _is_new_day(self):
        return datetime.datetime.now().date() > self._state_database.get("last update datetime").date()

    def get_interaction_type(self):
        logging.info("Determining interaction type")

        if self._is_first_interaction():
            interaction_type = Interactions.FIRST_INTERACTION
        elif self._is_time_for_scheduled_interaction():
            interaction_type = Interactions.SCHEDULED_INTERACTION
        elif self._is_ask_to_do_evaluation():
            interaction_type = Interactions.ASK_TO_DO_EVALUATION
        elif self._is_run_prompted_interaction():
            interaction_type = Interactions.PROMPTED_INTERACTION
        elif self._is_run_prompted_content():
            interaction_type = Interactions.PROMPTED_CONTENT
        elif self._is_off_checkin():
            interaction_type = Interactions.EVALUATION
        else:
            interaction_type = None
        return interaction_type

    def _is_first_interaction(self):
        return not self._state_database.is_set("first interaction datetime")

    def _is_time_for_scheduled_interaction(self):
        if not self._state_database.get("next checkin datetime"):
            return False

        if self._state_database.get("is done eval today"):
            is_time = False
        else:
            is_in_update_window = self._is_in_window(
                self._state_database.get("next checkin datetime"),
                self._update_window_seconds,
                datetime.datetime.now()
            )
            is_prompted_in_scheduled_window = self._is_in_window(
                self._state_database.get("next checkin datetime"),
                self._scheduled_window_minutes,
                datetime.datetime.now()
            )

            is_time = is_in_update_window or \
                (is_prompted_in_scheduled_window and not self._state_database.get("is done eval today") and
                 self._state_database.get("is prompted by user"))

        return is_time

    def _is_ask_to_do_evaluation(self):  # this graphs begins with 'ask to chat'
        is_in_scheduled_window = self._is_in_window(
            self._state_database.get("next checkin datetime"),
            self._scheduled_window_minutes,
            datetime.datetime.now()
        )
        return not self._state_database.get("is done eval today") and \
            self._state_database.get("is prompted by user") and \
            not is_in_scheduled_window

    def _is_run_prompted_interaction(self):
        return self._state_database.get("is prompted by user") and \
            self._state_database.get("is done eval today")

    def _is_run_prompted_content(self):
        return self._state_database.get("is prompted by user") and \
            self._state_database.get("is run prompted content")

    def _is_off_checkin(self):
        return self._state_database.get("is prompted by user") and \
               not self._state_database.get("is run prompted content") and \
               not self._state_database.get("is done eval today")

    def _is_in_window(self, center, window, time_to_check):
        start_time = center - window
        end_time = center + window
        return start_time <= time_to_check <= end_time

    def _reset_database(self):
        keys = self._state_database.get_keys()
        for key in keys:
            self._state_database.set(key, None)
