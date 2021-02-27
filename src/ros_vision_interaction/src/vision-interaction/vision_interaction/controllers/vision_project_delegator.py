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
        if datetime.datetime.now().day > self._state_database.get("last update datetime").day:
            self._state_database.set("is done evaluation today", False)
        self._state_database.set("last update datetime", datetime.datetime.now())

    def determine_interaction_type(self):
        logging.info("Determining interaction type")
        interaction_type = None

        if self._is_first_interaction():
            interaction_type = Interactions.FIRST_INTERACTION
        else:
            if self._state_database.get("is prompted by user"):
                self._state_database.set("is prompted by user", False)
                if not self._state_database.get("is done evaluation today"):
                    is_in_scheduled_window = self._is_in_window(
                        self._state_database.get("next checkin datetime"),
                        self._scheduled_window_minutes,
                        datetime.datetime.now()
                    )
                    if is_in_scheduled_window:
                        interaction_type = Interactions.SCHEDULED_INTERACTION
                    else:
                        interaction_type = Interactions.ASK_TO_DO_SCHEDULED
                else:
                    interaction_type = Interactions.PROMPTED_INTERACTION
            # if not prompted by user
            else:
                if self._is_off_checkin():
                    interaction_type = Interactions.EVALUATION
                elif self._state_database.get("is run prompted"):
                    self._state_database.set("is run prompted", False)
                    interaction_type = Interactions.PROMPTED_INTERACTION
                elif self._is_time_for_scheduled_interaction():
                    interaction_type = Interactions.SCHEDULED_INTERACTION
        return interaction_type

    def _is_first_interaction(self):
        return not self._state_database.is_set("first interaction datetime")

    def _is_time_for_scheduled_interaction(self):
        if not self._state_database.get("next checkin datetime"):
            return False

        if self._state_database.get("is done evaluation today"):
            is_time = False
        else:
            is_in_update_window = self._is_in_window(
                self._state_database.get("next checkin datetime"),
                self._update_window_seconds,
                datetime.datetime.now()
            )
            is_time = is_in_update_window

        return is_time

    def _is_in_window(self, center, window, time_to_check):
        start_time = center - window
        end_time = center + window
        return start_time <= time_to_check <= end_time

    def _reset_database(self):
        keys = self._state_database.get_keys()
        for key in keys:
            self._state_database.set(key, None)

    def _is_off_checkin(self):
        is_off_checkin = self._state_database.get("is off checkin")
        if is_off_checkin:
            self._state_database.set("is off checkin", False)
        return is_off_checkin
