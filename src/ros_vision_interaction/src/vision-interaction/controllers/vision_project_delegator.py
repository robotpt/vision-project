#!/usr/bin/env python3.8
import datetime
import logging
import math
import vision_project_tools.reading_task_tools as reading_task_tools

from vision_project_tools import increment_db_value
from vision_project_tools.constants import DatabaseKeys, INITIAL_STATE_DB, Interactions
from vision_project_tools.reading_task_tools import TaskDataKeys, Tasks

logging.basicConfig(level=logging.INFO)


class VisionProjectDelegator:

    def __init__(
            self,
            statedb,
            update_window_seconds=5,
            scheduled_window_minutes=15,
            minutes_between_interactions=1,
            max_num_of_prompted_per_day=3,
            score_window=10,
            is_reset_database=True
    ):
        self._state_database = statedb
        self._update_window_seconds = datetime.timedelta(seconds=update_window_seconds)
        self._scheduled_window_minutes = datetime.timedelta(minutes=scheduled_window_minutes)
        self._minutes_between_interactions = datetime.timedelta(minutes=minutes_between_interactions)
        self._max_num_of_prompted_per_day = max_num_of_prompted_per_day
        self._score_window = score_window

        self._is_run_interaction = False

        self._state_database.set(DatabaseKeys.LAST_UPDATE_DATETIME, datetime.datetime.now())
        self._state_database.set(DatabaseKeys.CURRENT_READING_INDEX, datetime.datetime.now().weekday())
        task_type = reading_task_tools.get_current_reading_task_type(self._state_database)
        self._state_database.set(DatabaseKeys.CURRENT_READING_TYPE, task_type)

        # for testing purposes
        if self._state_database.get("is first startup") or is_reset_database:
            print("Resetting database")
            self._reset_database()
            self._state_database.set(DatabaseKeys.LAST_UPDATE_DATETIME, datetime.datetime.now())
            self._state_database.set(DatabaseKeys.CURRENT_READING_INDEX, datetime.datetime.now().weekday())
            task_type = reading_task_tools.get_current_reading_task_type(self._state_database)
            self._state_database.set(DatabaseKeys.CURRENT_READING_TYPE, task_type)
            self.new_day_update()

    def update(self):
        if self._is_new_day():
            self.new_day_update()
        self._state_database.set(DatabaseKeys.LAST_UPDATE_DATETIME, datetime.datetime.now())

    def increment_system_date(self):
        date_times = [
            DatabaseKeys.FIRST_INTERACTION_DATETIME,
            DatabaseKeys.LAST_INTERACTION_DATETIME,
            DatabaseKeys.NEXT_CHECKIN_DATETIME
        ]
        for key in date_times:
            previous_datetime = self._state_database.get(key)
            try:
                self._state_database.set(key, self._decrement_date(previous_datetime))
            except TypeError:
                logging.info(f"{key} was not able to be changed.")
        self.new_day_update()

    def _decrement_date(self, date_time):
        return date_time - datetime.timedelta(days=1)

    def _is_new_day(self):
        return datetime.datetime.now().date() > self._state_database.get(DatabaseKeys.LAST_UPDATE_DATETIME).date()

    def new_day_update(self):
        self._daily_reading_task_data_update()
        self._daily_state_update()
        self._update_act_variables()
        self._state_database.set(DatabaseKeys.NUM_OF_PROMPTED_TODAY, 0)
        self._state_database.set(DatabaseKeys.PERSEVERANCE_COUNTER, 0)
        self._state_database.set(DatabaseKeys.FEELINGS_INDEX, None)
        self._state_database.set(DatabaseKeys.IS_PUBLISHED_CHOICES_TODAY, False)
        self._state_database.set(DatabaseKeys.VIDEO_TO_PLAY, None)
        increment_db_value(self._state_database, DatabaseKeys.INTERACTION_DAY)

    def _daily_state_update(self):
        keys_to_check = {
            DatabaseKeys.IS_DONE_EVAL_TODAY: DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL,
            DatabaseKeys.IS_DONE_PROMPTED_TODAY: DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT,
            DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE,
            DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS,
            DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING
        }
        for key in keys_to_check:
            if not self._state_database.get(key):
                increment_db_value(self._state_database, keys_to_check[key])
            else:
                self._state_database.set(keys_to_check[key], 0)
            self._state_database.set(key, False)
        # make sure there is a scheduled session
        next_checkin_datetime = self._state_database.get(DatabaseKeys.NEXT_CHECKIN_DATETIME)
        if next_checkin_datetime is not None and next_checkin_datetime.date() < datetime.datetime.now().date():
            delta = datetime.timedelta(days=1)
            next_checkin_datetime = next_checkin_datetime + delta
            self._state_database.set(
                DatabaseKeys.NEXT_CHECKIN_DATETIME,
                next_checkin_datetime.replace(day=next_checkin_datetime.day + 1)
            )

    def _update_act_variables(self):
        last_5_scores = self._state_database.get(DatabaseKeys.LAST_5_EVAL_SCORES)
        if len(last_5_scores) == 5:
            last_5_scores.pop(0)
        last_5_scores.append(self._state_database.get(DatabaseKeys.CURRENT_EVAL_SCORE))
        self._state_database.set(DatabaseKeys.LAST_5_EVAL_SCORES, last_5_scores)

    def _daily_reading_task_data_update(self):
        # set reading task data for the new day
        current_type = self._state_database.get(DatabaseKeys.CURRENT_READING_TYPE)
        current_id = self._state_database.get(DatabaseKeys.CURRENT_READING_ID)
        current_score = reading_task_tools.get_reading_task_data_value(
            self._state_database,
            current_id,
            reading_task_tools.TaskDataKeys.ANNOTATOR_SCORE
        )
        if current_score is None:
            current_score = reading_task_tools.get_reading_task_data_value(
                self._state_database,
                current_id,
                reading_task_tools.TaskDataKeys.SCORE
            )
        best_scores = self._state_database.get(DatabaseKeys.BEST_SCORES)
        last_scores = self._state_database.get(DatabaseKeys.LAST_SCORES)

        try:
            if current_score > best_scores[current_type]:
                best_scores[current_type] = current_score
                self._state_database.set(DatabaseKeys.BEST_SCORES, best_scores)
            last_scores[current_type] = current_score
            self._state_database.set(DatabaseKeys.LAST_SCORES, last_scores)
        except TypeError:
            logging.info("Last and best scores are not set, skipping update")

        # update reading task index if reading task has been done
        if self._state_database.get(DatabaseKeys.IS_DONE_EVAL_TODAY):
            current_index = self._state_database.get(DatabaseKeys.CURRENT_READING_INDEX)
            new_index = (current_index + 1) % 6
            self._state_database.set(DatabaseKeys.CURRENT_READING_INDEX, new_index)

        new_id = reading_task_tools.get_new_day_reading_task(self._state_database)
        self._state_database.set(DatabaseKeys.CURRENT_READING_ID, new_id)
        task_color = reading_task_tools.get_reading_task_data_value(self._state_database, new_id, TaskDataKeys.COLOR)
        self._state_database.set(DatabaseKeys.CURRENT_READING_COLOR, task_color)

        # set values needed for QT evaluation feedback
        new_type = reading_task_tools.get_current_reading_task_type(self._state_database)
        last_score = self._state_database.get(DatabaseKeys.LAST_SCORES)[new_type]
        best_score = self._state_database.get(DatabaseKeys.BEST_SCORES)[new_type]
        self._state_database.set(DatabaseKeys.LAST_SCORE, last_score)
        self._state_database.set(DatabaseKeys.BEST_SCORE, best_score)

    def get_interaction_type(self):
        logging.info("Determining interaction type")
        if self._is_first_interaction():
            interaction_type = Interactions.FIRST_INTERACTION
        elif self._is_time_for_scheduled_interaction():
            interaction_type = Interactions.SCHEDULED_INTERACTION
        elif self._is_run_too_many_prompted():
            interaction_type = Interactions.TOO_MANY_PROMPTED
        elif self._is_run_prompted_interaction():
            interaction_type = Interactions.PROMPTED_INTERACTION
        elif self._is_ask_to_do_scheduled():
            interaction_type = Interactions.ASK_TO_DO_SCHEDULED
        else:
            interaction_type = None
        return interaction_type

    def _is_first_interaction(self):
        return not self._state_database.is_set(DatabaseKeys.FIRST_INTERACTION_DATETIME)

    def _is_time_for_scheduled_interaction(self):
        if not self._state_database.get(DatabaseKeys.NEXT_CHECKIN_DATETIME):
            return False

        if self._state_database.get(DatabaseKeys.IS_DONE_EVAL_TODAY):
            is_time = False
        else:
            is_in_update_window = self._is_in_window(
                self._state_database.get(DatabaseKeys.NEXT_CHECKIN_DATETIME),
                self._update_window_seconds,
                datetime.datetime.now()
            )
            is_prompted_in_scheduled_window = self._is_in_window(
                self._state_database.get(DatabaseKeys.NEXT_CHECKIN_DATETIME),
                self._scheduled_window_minutes,
                datetime.datetime.now()
            )

            is_time = is_in_update_window or \
                (is_prompted_in_scheduled_window and not
                    self._state_database.get(DatabaseKeys.IS_DONE_EVAL_TODAY) and
                    self._state_database.get(DatabaseKeys.IS_PROMPTED_BY_USER)
                 )

        return is_time

    def _is_ask_to_do_scheduled(self):
        is_in_scheduled_window = self._is_in_window(
            self._state_database.get(DatabaseKeys.NEXT_CHECKIN_DATETIME),
            self._scheduled_window_minutes,
            datetime.datetime.now()
        )
        return self._state_database.get(DatabaseKeys.IS_PROMPTED_BY_USER) and \
               not self._state_database.get(DatabaseKeys.IS_DONE_EVAL_TODAY) and \
               not is_in_scheduled_window

    def _is_run_prompted_interaction(self):
        return self._state_database.get(DatabaseKeys.IS_PROMPTED_BY_USER) and \
               self._state_database.get(DatabaseKeys.IS_DONE_EVAL_TODAY) and \
               self._state_database.get(DatabaseKeys.NUM_OF_PROMPTED_TODAY) < self._max_num_of_prompted_per_day

    def _is_run_too_many_prompted(self):
        return self._state_database.get(DatabaseKeys.IS_PROMPTED_BY_USER) and \
               self._state_database.get(DatabaseKeys.NUM_OF_PROMPTED_TODAY) >= self._max_num_of_prompted_per_day

    def _is_in_window(self, center, window, time_to_check):
        start_time = center - window
        end_time = center + window
        return start_time <= time_to_check <= end_time

    def _reset_database(self):
        for key in INITIAL_STATE_DB:
            self._state_database.set(key, INITIAL_STATE_DB[key])
