#!/usr/bin/python3.8
import datetime
import freezegun
import pytest
import vision_project_tools.reading_task_tools as reading_task_tools

from vision_project_tools.constants import DatabaseKeys, READING_TASK_DATA
from vision_project_tools.reading_task_tools import Tasks, TaskDataKeys


def reset_reading_task_data(statedb):
    statedb.set(DatabaseKeys.READING_TASK_DATA, READING_TASK_DATA)


def test_get_current_reading_task_type(statedb):
    dates = [
        "2021-07-11 00:00:00",
        "2021-07-12 00:00:00",
        "2021-07-13 00:00:00",
        "2021-07-14 00:00:00",
        "2021-07-15 00:00:00",
        "2021-07-16 00:00:00",
        "2021-07-17 00:00:00",
        "2021-07-18 00:00:00",
    ]
    expected_task_types = [
        [Tasks.MNREAD, Tasks.SKREAD],
        [Tasks.SPOT_READING],
        [Tasks.SRT],
        [Tasks.SPOT_READING],
        [Tasks.IREST],
        [Tasks.SPOT_READING],
        [Tasks.SRT],
        [Tasks.MNREAD, Tasks.SKREAD]
    ]
    for i, date in enumerate(dates):
        with freezegun.freeze_time(date):
            assert reading_task_tools.get_current_reading_task_type(statedb) in expected_task_types[i]


def test_get_current_reading_task_id(statedb):

    def is_task_completed(reading_task_data, task_id):
        for task_type in reading_task_data:
            # for difficulty_level in reading_task_data[task_type]:
            #     if task_id in reading_task_data[task_type][difficulty_level].keys():
            #         return reading_task_data[task_type][difficulty_level][task_id]["score"] is not None
            if task_id in reading_task_data[task_type].keys():
                return reading_task_data[task_type][task_id]["score"] is not None

    dates = [
        "2021-07-11 00:00:00",
        "2021-07-12 00:00:00",
        "2021-07-13 00:00:00",
        "2021-07-14 00:00:00",
        "2021-07-15 00:00:00",
        "2021-07-16 00:00:00",
        "2021-07-17 00:00:00",
        "2021-07-18 00:00:00",
    ]
    # statedb.set(DatabaseKeys.DIFFICULTY_LEVEL, "1")
    for i, date in enumerate(dates):
        with freezegun.freeze_time(date):
            task_id = reading_task_tools.get_new_day_reading_task(statedb)
            assert not is_task_completed(READING_TASK_DATA, task_id)


def test_set_reading_score(statedb):
    reading_scores = [10, 12, 14, 15, 17, 19, 20, 21]
    dates = [
        "2021-07-11 00:00:00",
        "2021-07-12 00:00:00",
        "2021-07-13 00:00:00",
        "2021-07-14 00:00:00",
        "2021-07-15 00:00:00",
        "2021-07-16 00:00:00",
        "2021-07-17 00:00:00",
        "2021-07-18 00:00:00",
    ]
    # statedb.set(DatabaseKeys.DIFFICULTY_LEVEL, "1")

    for i, date in enumerate(dates):
        with freezegun.freeze_time(date):
            task_id = reading_task_tools.get_new_day_reading_task(statedb)
            reading_score = reading_scores[i]
            reading_task_tools.set_reading_task_value(statedb, task_id, TaskDataKeys.SCORE, reading_score)
            assert reading_task_tools.get_reading_task_data_value(statedb, task_id, TaskDataKeys.SCORE) == reading_score
            reset_reading_task_data(statedb)
