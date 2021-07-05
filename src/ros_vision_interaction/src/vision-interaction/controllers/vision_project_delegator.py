#!/usr/bin/env python3.8
import datetime
import logging
import math

from controllers.interaction_manager import Interactions

logging.basicConfig(level=logging.INFO)


class DatabaseKeys:
    BEST_SCORE = "best score"
    CURRENT_EVAL_SCORE = "current eval score"
    CURRENT_READING_COLOR = "current reading color"
    CURRENT_READING_ID = "current reading id"
    DIFFICULTY_LEVEL = "difficulty level"
    FEEDBACK_VIDEOS = "feedback videos"
    FIRST_INTERACTION_DATETIME = "first interaction datetime"
    GOOD_TO_CHAT = "good to chat"
    GRIT_FEEDBACK_INDEX = "grit feedback index"
    IS_CONTINUE_PERSEVERANCE = "is continue perseverance"
    IS_DO_EVALUATION = "is do evaluation"
    IS_DONE_EVAL_TODAY = "is done eval today"
    IS_DONE_GOAL_SETTING_TODAY = "is done goal setting today"
    IS_DONE_MINDFULNESS_TODAY = "is done mindfulness today"
    IS_DONE_PERSEVERANCE_TODAY = "is done perseverance today"
    IS_DONE_PROMPTED_TODAY = "is done prompted today"
    IS_INTERACTION_FINISHED = "is interaction finished"
    IS_NEW_DIFFICULTY_LEVEL = "is new difficulty level"
    IS_OFF_CHECKIN = "is off checkin"
    IS_PROMPTED_BY_USER = "is prompted by user"
    IS_PUBLISHED_CHOICES_TODAY = "is published choices today"
    IS_START_PERSEVERANCE = "is start perseverance"
    IS_USED_MAGNIFIER_TODAY = "is used magnifier today"
    LAST_5_EVAL_SCORES = "last 5 eval scores"
    LAST_INTERACTION_DATETIME = "last interaction datetime"
    LAST_UPDATE_DATETIME = "last update datetime"
    LAST_SCORE = "last score"
    FEELINGS_INDEX = "feelings index"
    NEXT_CHECKIN_DATETIME = "next checkin datetime"
    NUM_OF_DAYS_SINCE_LAST_EVAL = "num of days since last eval"
    NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING = "num of days since last goal setting"
    NUM_OF_DAYS_SINCE_LAST_MINDFULNESS = "num of days since last mindfulness"
    NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE = "num of days since last perseverance"
    NUM_OF_DAYS_SINCE_LAST_PROMPT = "num of days since last prompt"
    NUM_OF_PROMPTED_TODAY = "num of prompted today"
    PERSEVERANCE_COUNTER = "perseverance counter"
    READING_EVAL_DATA = "reading eval data"
    READING_EVAL_INDEX = "reading eval index"
    READING_EVAL_TYPE = "reading eval type"
    USER_NAME = "user name"
    VIDEO_TO_PLAY = "video to play"


INITIAL_STATE_DB = {
    DatabaseKeys.BEST_SCORE: None,
    DatabaseKeys.CURRENT_EVAL_SCORE: 0,
    DatabaseKeys.CURRENT_READING_COLOR: None,
    DatabaseKeys.CURRENT_READING_ID: None,
    DatabaseKeys.DIFFICULTY_LEVEL: 1,
    DatabaseKeys.FEEDBACK_VIDEOS: {
        "video 1": "https://www.youtube.com/embed/4b33NTAuF5E",
        "video 2": "https://www.youtube.com/embed/JXeJANDKwDc",
        "video 3": "https://www.youtube.com/embed/uzkD5SeuwzM",
        "video 4": "https://www.youtube.com/embed/QqsLTNkzvaY",
        "no video": ""
    },
    DatabaseKeys.FIRST_INTERACTION_DATETIME: None,
    DatabaseKeys.GOOD_TO_CHAT: None,
    DatabaseKeys.GRIT_FEEDBACK_INDEX: 0,
    DatabaseKeys.IS_CONTINUE_PERSEVERANCE: None,
    DatabaseKeys.IS_DO_EVALUATION: None,
    DatabaseKeys.IS_DONE_EVAL_TODAY: False,
    DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
    DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
    DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
    DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
    DatabaseKeys.IS_INTERACTION_FINISHED: False,
    DatabaseKeys.IS_NEW_DIFFICULTY_LEVEL: False,
    DatabaseKeys.IS_OFF_CHECKIN: None,
    DatabaseKeys.IS_PROMPTED_BY_USER: False,
    DatabaseKeys.IS_PUBLISHED_CHOICES_TODAY: False,
    DatabaseKeys.IS_START_PERSEVERANCE: False,
    DatabaseKeys.IS_USED_MAGNIFIER_TODAY: False,
    DatabaseKeys.LAST_5_EVAL_SCORES: [],
    DatabaseKeys.LAST_INTERACTION_DATETIME: None,
    DatabaseKeys.LAST_UPDATE_DATETIME: None,
    DatabaseKeys.LAST_SCORE: None,
    DatabaseKeys.FEELINGS_INDEX: None,
    DatabaseKeys.NEXT_CHECKIN_DATETIME: None,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
    DatabaseKeys.NUM_OF_PROMPTED_TODAY: 0,
    DatabaseKeys.PERSEVERANCE_COUNTER: 0,
    DatabaseKeys.READING_EVAL_DATA: [],
    DatabaseKeys.READING_EVAL_INDEX: 0,
    DatabaseKeys.READING_EVAL_TYPE: None,
    DatabaseKeys.USER_NAME: "",
    DatabaseKeys.VIDEO_TO_PLAY: None
}


class VisionProjectDelegator:

    def __init__(
            self,
            statedb,
            update_window_seconds=5,
            scheduled_window_minutes=15,
            minutes_between_interactions=1,
            max_num_of_prompted_per_day=3,
            score_window=10
    ):
        self._state_database = statedb
        self._update_window_seconds = datetime.timedelta(seconds=update_window_seconds)
        self._scheduled_window_minutes = datetime.timedelta(minutes=scheduled_window_minutes)
        self._minutes_between_interactions = datetime.timedelta(minutes=minutes_between_interactions)
        self._max_num_of_prompted_per_day = max_num_of_prompted_per_day
        self._score_window = score_window

        self._is_run_interaction = False

        # for testing purposes
        self._reset_database()

        self._state_database.set("last update datetime", datetime.datetime.now())

    def update(self):
        if not self._state_database.is_set("last update datetime"):
            self._state_database.set("last update datetime", datetime.datetime.now())
        if self._is_new_day():
            self._daily_state_update()
            self._update_act_variables()
            self._state_database.set("num of prompted today", 0)
            self._state_database.set("perseverance counter", 0)
            self._state_database.set("feelings index", None)
            self._state_database.set("is published choices today", False)
        self._state_database.set("last update datetime", datetime.datetime.now())

    def _is_new_day(self):
        return datetime.datetime.now().date() > self._state_database.get("last update datetime").date()

    def _daily_state_update(self):
        keys_to_check = {
            "is done eval today": "num of days since last eval",
            "is done prompted today": "num of days since last prompt",
            "is done perseverance today": "num of days since last perseverance",
            "is done mindfulness today": "num of days since last mindfulness",
            "is done goal setting today": "num of days since last goal setting"
        }
        for key in keys_to_check:
            if not self._state_database.get(key):
                old_value = self._state_database.get(keys_to_check[key])
                self._state_database.set(keys_to_check[key], old_value + 1)
            else:
                self._state_database.set(keys_to_check[key], 0)
            self._state_database.set(key, False)

    def _update_act_variables(self):
        last_5_scores = self._state_database.get("last 5 eval scores")
        if len(last_5_scores) == 5:
            last_5_scores.pop(0)
        last_5_scores.append(self._state_database.get("current eval score"))
        self._state_database.set("last 5 eval scores", last_5_scores)

    def _update_reading_task_data(self):
        last_5_scores = self._state_database.get("last 5 eval scores")
        average_score = sum(last_5_scores)/len(last_5_scores)
        last_score = self._state_database.get("last score")
        if average_score - self._score_window <= last_score <= average_score + self._score_window:
            self._state_database.set("grit feedback index", 0)  # STABLE
        elif last_score < average_score - self._score_window:
            self._state_database.set("grit feedback index", 1)  # DECLINED
        else:
            self._state_database.set("grit feedback index", 2)  # IMPROVED

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

    def _is_ask_to_do_scheduled(self):
        is_in_scheduled_window = self._is_in_window(
            self._state_database.get("next checkin datetime"),
            self._scheduled_window_minutes,
            datetime.datetime.now()
        )
        return self._state_database.get("is prompted by user") and \
            not self._state_database.get("is done eval today") and \
            not is_in_scheduled_window

    def _is_run_prompted_interaction(self):
        return self._state_database.get("is prompted by user") and \
            self._state_database.get("is done eval today") and \
            self._state_database.get("num of prompted today") < self._max_num_of_prompted_per_day

    def _is_run_too_many_prompted(self):
        return self._state_database.get("is prompted by user") and \
               self._state_database.get("num of prompted today") >= self._max_num_of_prompted_per_day

    def _is_in_window(self, center, window, time_to_check):
        start_time = center - window
        end_time = center + window
        return start_time <= time_to_check <= end_time

    def _reset_database(self):
        for key in INITIAL_STATE_DB:
            self._state_database.set(key, INITIAL_STATE_DB[key])
