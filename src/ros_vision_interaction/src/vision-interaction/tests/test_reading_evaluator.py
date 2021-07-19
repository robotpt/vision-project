#!/usr/bin/python3.8
import datetime
import freezegun
import mock
import pytest
import vision_project_tools.reading_task_tools as reading_task_tools

from reading_evaluator import ReadingEvaluator
from vision_project_tools.constants import DatabaseKeys
from vision_project_tools.reading_task_tools import Tasks, TaskDataKeys


@pytest.fixture
def reading_evaluator(statedb):
    return ReadingEvaluator(statedb)


def test_set_reading_speed(statedb, reading_evaluator):
    ids = ["ir11", "mn11", "sk11", "sr11", "sr22", "srt11"]
    statedb.set(DatabaseKeys.DIFFICULTY_LEVEL, "1")
    with mock.patch("reading_evaluator.ReadingEvaluator.get_total_speaking_time") as mock_get_total_time:
        for task_id in ids:
            current_id = task_id
            statedb.set(DatabaseKeys.CURRENT_READING_ID, current_id)
            total_time = 100
            mock_get_total_time.return_value = total_time
            reading_evaluator.calculate_and_set_reading_score("temp.wav")

            word_count = reading_task_tools.get_reading_task_data_value(statedb, current_id, TaskDataKeys.WORD_COUNT)
            expected_score = word_count / total_time
            actual_score = reading_task_tools.get_reading_task_data_value(statedb, current_id, TaskDataKeys.SCORE)
            assert actual_score == expected_score
