#!/usr/bin/python3.8
import datetime
import freezegun
import math
import pytest

from controllers import VisionProjectDelegator
from vision_project_tools.constants import DatabaseKeys, Interactions


@pytest.fixture
def vision_project_delegator(statedb):
    # TODO: make params env variables
    return VisionProjectDelegator(
        statedb,
        update_window_seconds=15,
        minutes_between_interactions=1
    )


def check_database_values(statedb, **kwargs):
    for key, value in kwargs.items():
        assert statedb.get(key) == value


def test_determine_first_interaction(vision_project_delegator, statedb):
    assert vision_project_delegator._is_first_interaction()
    assert not vision_project_delegator._is_time_for_scheduled_interaction()
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime(2021, 1, 15, 6, 0, 0))
    assert not vision_project_delegator._is_first_interaction()


def test_determine_scheduled_interaction_not_prompted(vision_project_delegator, statedb):
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set(DatabaseKeys.LAST_INTERACTION_DATETIME, datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set(DatabaseKeys.IS_PROMPTED_BY_USER, False)
    statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, False)
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set(DatabaseKeys.NEXT_CHECKIN_DATETIME, next_checkin_time)

    scheduled_window_times = [
        "2021-1-12 11:59:46",
        "2021-1-12 12:00:00",
        "2021-1-12 12:00:14",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:15",
    ]
    # check that scheduled is called when in time window
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator._is_time_for_scheduled_interaction()
            assert vision_project_delegator.get_interaction_type() == Interactions.SCHEDULED_INTERACTION

    outside_scheduled_window_times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 12:00:16"
    ]
    # check scheduled is not called when not in time window
    for invalid_time in outside_scheduled_window_times:
        with freezegun.freeze_time(invalid_time):
            assert not vision_project_delegator._is_time_for_scheduled_interaction()


def test_determine_interaction_prompted_no_evaluation(vision_project_delegator, statedb):
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set(DatabaseKeys.LAST_INTERACTION_DATETIME, datetime.datetime(2021, 1, 10, 6, 0, 0))
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set(DatabaseKeys.NEXT_CHECKIN_DATETIME, next_checkin_time)
    statedb.set(DatabaseKeys.IS_PROMPTED_BY_USER, True)
    statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, False)
    # check scheduled interaction is called when within scheduled interaction window and evaluation not done yet
    # window of +/- 15 minutes
    scheduled_window_times = [
        "2021-1-12 11:45:00",
        "2021-1-12 11:46:00",
        "2021-1-12 12:00:00",
        "2021-1-12 12:14:00",
        "2021-1-12 12:15:00",
    ]
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator.get_interaction_type() == Interactions.SCHEDULED_INTERACTION
    # check 'ask to run prompted' called when not within scheduled interaction window and evaluation not done yet
    scheduled_window_times = [
        "2021-1-12 11:44:59",
        "2021-1-12 12:15:01",
        "2021-1-12 11:00:00",
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
    ]
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator.get_interaction_type() == Interactions.ASK_TO_DO_SCHEDULED


def test_determine_interaction_prompted_evaluation_done(vision_project_delegator, statedb):
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set(DatabaseKeys.LAST_INTERACTION_DATETIME, datetime.datetime(2021, 1, 10, 6, 0, 0))
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set(DatabaseKeys.NEXT_CHECKIN_DATETIME, next_checkin_time)
    statedb.set(DatabaseKeys.IS_PROMPTED_BY_USER, True)
    statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
    # check that prompted interaction is called when evaluation has been done and QT is prompted
    times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:00",
        "2021-1-12 12:00:15",
        "2021-1-12 12:00:16"
    ]
    for time in times:
        with freezegun.freeze_time(time):
            assert vision_project_delegator.get_interaction_type() == Interactions.PROMPTED_INTERACTION


def test_is_done_reading_evaluation(vision_project_delegator, statedb):
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime(2021, 2, 9, 6, 0, 0))
    statedb.set(DatabaseKeys.NEXT_CHECKIN_DATETIME, datetime.datetime(2021, 2, 11, 2, 0, 0))
    statedb.set(DatabaseKeys.LAST_UPDATE_DATETIME, datetime.datetime(2021, 2, 10, 12, 0, 0))
    statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL, 0)

    # check scheduled interaction is called when valid time and evaluation not done yet
    with freezegun.freeze_time("2021-02-11 2:00:00"):
        assert statedb.get(DatabaseKeys.LAST_UPDATE_DATETIME) == datetime.datetime(2021, 2, 10, 12, 0, 0)
        vision_project_delegator.update()
        assert not statedb.get(DatabaseKeys.IS_DONE_EVAL_TODAY)
        assert vision_project_delegator.get_interaction_type() == Interactions.SCHEDULED_INTERACTION

        # check no interaction is called when valid time but evaluation already done
        with freezegun.freeze_time("2021-02-11 2:00:00"):
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            assert vision_project_delegator.get_interaction_type() is None


def test_too_many_prompts(vision_project_delegator, statedb):
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime(2021, 2, 9, 6, 0, 0))
    statedb.set(DatabaseKeys.LAST_UPDATE_DATETIME, datetime.datetime(2021, 2, 10, 12, 0, 0))
    next_checkin_time = datetime.datetime(2021, 2, 10, 12, 0, 0)
    statedb.set(DatabaseKeys.NEXT_CHECKIN_DATETIME, next_checkin_time)
    statedb.set(DatabaseKeys.NUM_OF_PROMPTED_TODAY, 3)
    statedb.set(DatabaseKeys.IS_PROMPTED_BY_USER, True)
    statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
    assert vision_project_delegator.get_interaction_type() == Interactions.TOO_MANY_PROMPTED


def test_new_day_update(vision_project_delegator, statedb):
    test_periods = [
        [
            "2021-03-15 00:00:00",  # Monday
            "2021-03-16 00:00:00",
            "2021-03-17 00:00:00",
            "2021-03-18 00:00:00",
            "2021-03-19 00:00:00",
            "2021-03-20 00:00:00",
            "2021-03-21 00:00:00",
        ],
        # month changes during test period
        [
            "2021-02-25 00:00:00",  # Thursday
            "2021-02-26 00:00:00",
            "2021-02-27 00:00:00",
            "2021-02-28 00:00:00",
            "2021-03-01 00:00:00",
            "2021-03-02 00:00:00",
            "2021-03-03 00:00:00",
        ],
        # year changes during test period
        [
            "2020-12-29 00:00:00",  # Tuesday
            "2020-12-30 00:00:00",
            "2020-12-31 00:00:00",
            "2021-01-01 00:00:00",
            "2021-01-02 00:00:00",
            "2021-01-03 00:00:00",
            "2021-01-04 00:00:00",
        ]
    ]
    for period in test_periods:
        index = None
        vision_project_delegator._reset_database()
        # day 1
        with freezegun.freeze_time(period[0]):
            index = datetime.datetime.now().weekday()
            vision_project_delegator.update()
            first_interaction_datetime = datetime.datetime(2021, 3, 15, 2, 0, 0, 0)
            statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, first_interaction_datetime)
            statedb.set(DatabaseKeys.LAST_INTERACTION_DATETIME, first_interaction_datetime)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_READING_INDEX, index)
        # day 2
        with freezegun.freeze_time(period[1]):
            index = int(math.fmod(index + 1, 6))
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 1,
                DatabaseKeys.CURRENT_READING_INDEX: index
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, False)
        # day 3
        with freezegun.freeze_time(period[2]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 2,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 2,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 2,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 2,
                DatabaseKeys.CURRENT_READING_INDEX: index
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
        # day 4
        with freezegun.freeze_time(period[3]):
            index = int(math.fmod(index + 1, 6))
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.CURRENT_READING_INDEX: index
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
        # day 5
        with freezegun.freeze_time(period[4]):
            index = int(math.fmod(index + 1, 6))
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.CURRENT_READING_INDEX: index
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
        # day 6
        with freezegun.freeze_time(period[5]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.CURRENT_READING_INDEX: index
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, False)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
        # day 7
        with freezegun.freeze_time(period[6]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 2,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 2,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.CURRENT_READING_INDEX: index
            }
            check_database_values(statedb, **expected_values)


def test_reading_evaluator(vision_project_delegator, statedb):
    test_periods = [
        [
            "2021-03-15 00:00:00",
            "2021-03-16 00:00:00",
            "2021-03-17 00:00:00",
            "2021-03-18 00:00:00",
            "2021-03-19 00:00:00",
            "2021-03-20 00:00:00",
            "2021-03-21 00:00:00",
            "2021-03-22 00:00:00",
            "2021-03-23 00:00:00",
        ],
        # month changes during test period
        [
            "2021-02-25 00:00:00",
            "2021-02-26 00:00:00",
            "2021-02-27 00:00:00",
            "2021-02-28 00:00:00",
            "2021-03-01 00:00:00",
            "2021-03-02 00:00:00",
            "2021-03-03 00:00:00",
            "2021-03-04 00:00:00",
            "2021-03-05 00:00:00",
        ],
        # year changes during test period
        [
            "2020-12-29 00:00:00",
            "2020-12-30 00:00:00",
            "2020-12-31 00:00:00",
            "2021-01-01 00:00:00",
            "2021-01-02 00:00:00",
            "2021-01-03 00:00:00",
            "2021-01-04 00:00:00",
            "2021-01-05 00:00:00",
            "2021-01-06 00:00:00",
        ]
    ]
    for period in test_periods:
        vision_project_delegator._reset_database()
        # day 1
        with freezegun.freeze_time(period[0]):
            vision_project_delegator.update()
            first_interaction_datetime = datetime.datetime(2021, 3, 15, 2, 0, 0, 0)
            statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, first_interaction_datetime)
            statedb.set(DatabaseKeys.LAST_INTERACTION_DATETIME, first_interaction_datetime)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 1)
        # day 2
        with freezegun.freeze_time(period[1]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 1,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 1,
                DatabaseKeys.LAST_5_EVAL_SCORES: [1],
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 3)
        # day 3
        with freezegun.freeze_time(period[2]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.LAST_5_EVAL_SCORES: [1, 3],
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 2)
        # day 4
        with freezegun.freeze_time(period[3]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.LAST_5_EVAL_SCORES: [1, 3, 2],
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 5)
        # day 5
        with freezegun.freeze_time(period[4]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.LAST_5_EVAL_SCORES: [1, 3, 2, 5],
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 3)
        # day 6
        with freezegun.freeze_time(period[5]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.LAST_5_EVAL_SCORES: [1, 3, 2, 5, 3],
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 3)
        # day 7
        with freezegun.freeze_time(period[6]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.LAST_5_EVAL_SCORES: [3, 2, 5, 3, 3],
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 5)
        # day 8
        with freezegun.freeze_time(period[7]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.LAST_5_EVAL_SCORES: [2, 5, 3, 3, 5],
            }
            check_database_values(statedb, **expected_values)
            statedb.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_MINDFULNESS_TODAY, True)
            statedb.set(DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY, True)
            statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 1)
        # day 9
        with freezegun.freeze_time(period[8]):
            vision_project_delegator.update()
            expected_values = {
                DatabaseKeys.IS_DONE_EVAL_TODAY: False,
                DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
                DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
                DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
                DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
                DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
                DatabaseKeys.LAST_5_EVAL_SCORES: [5, 3, 3, 5, 1],
            }
            check_database_values(statedb, **expected_values)
