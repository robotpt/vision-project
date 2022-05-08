import datetime
import logging
import random

from vision_project_tools.constants import DatabaseKeys

logging.basicConfig(level=logging.INFO)


class Tasks:
    IREST = "IReST"
    MNREAD = "MNread"
    SELF_SELECTED = "self selected"
    SKREAD = "SKread"
    SPOT_READING = "spot reading"
    SRT = "SRT"
    MN_IREST = "MN_IREST"


class TaskDataKeys:
    ANNOTATOR_SCORE = "annotator_score"
    ANSWER = "answer"
    COLOR = "color"
    CORRECT = "correct"
    IS_SCHEDULED = "is scheduled"
    SCORE = "score"
    UNABLE_TO_READ = "unable to read"
    WORD_COUNT = "word count"


def get_new_day_reading_task(statedb):
    task_type = get_current_reading_task_type(statedb)
    # current_difficulty_level = statedb.get(DatabaseKeys.DIFFICULTY_LEVEL)
    return get_reading_task_id(statedb, task_type)


def get_current_reading_task_type(statedb):
    reading_index = statedb.get(DatabaseKeys.CURRENT_READING_INDEX)
    if reading_index in [0, 14, 21, 28]:
        if not statedb.get(DatabaseKeys.IS_DONE_EVAL_TODAY):
            task_type = Tasks.MNREAD
        else:
            task_type = Tasks.SPOT_READING
    elif reading_index in [1, 8, 15, 22, 29, 36]:
        if not statedb.get(DatabaseKeys.IS_DONE_EVAL_TODAY):
            task_type = Tasks.IREST
        else:
            task_type = Tasks.SPOT_READING
    elif reading_index in [2, 9, 16, 23, 30, 37, 4, 11, 18, 25, 32, 39, 6, 13, 20, 27, 34]:
        task_type = Tasks.SRT
    elif reading_index in [3, 10, 17, 24, 31, 38, 5, 12, 19, 26, 33, 40]:
        task_type = Tasks.SPOT_READING
    elif reading_index in [7, 35]:
        if not statedb.get(DatabaseKeys.IS_DONE_EVAL_TODAY):
            task_type = Tasks.SKREAD
        else:
            task_type = Tasks.SPOT_READING
    elif reading_index == 41:
        task_type = Tasks.MN_IREST
    else:
        logging.info(f"No tasks for weekday index {reading_index}, setting task type to 'spot reading'.")
        task_type = Tasks.SPOT_READING
    return task_type


def get_reading_task_id(statedb, task_type, difficulty_level=None):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    # tasks = reading_task_data[task_type][difficulty_level]
    tasks = reading_task_data[task_type]
    if task_type == Tasks.SRT:
        index = statedb.get(DatabaseKeys.SRT_READING_INDEX)
        result = list(tasks.keys())[index]
    elif task_type == Tasks.IREST:
        index = statedb.get(DatabaseKeys.IREST_READING_INDEX)
        result = list(tasks.keys())[index]
    elif task_type == Tasks.SPOT_READING:
        index = statedb.get(DatabaseKeys.SPOT_READING_INDEX)
        result = list(tasks.keys())[index]
    elif task_type == Tasks.MNREAD:
        index = statedb.get(DatabaseKeys.MNREAD_INDEX)
        result = list(tasks.keys())[index]
    else:  # SKread
        index = statedb.get(DatabaseKeys.SKREAD_INDEX)
        result = list(tasks.keys())[index]
    return result


def get_reading_task_data_value(statedb, task_id, data_type):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    if data_type not in [
        TaskDataKeys.ANNOTATOR_SCORE,
        TaskDataKeys.ANSWER,
        TaskDataKeys.COLOR,
        TaskDataKeys.CORRECT,
        TaskDataKeys.SCORE,
        TaskDataKeys.UNABLE_TO_READ,
        TaskDataKeys.WORD_COUNT
    ]:
        raise KeyError(f"'{data_type}' is not a valid reading task data type.")
    for task_type in reading_task_data:
        # for difficulty_level in reading_task_data[task_type]:
        #     if task_id in reading_task_data[task_type][difficulty_level].keys():
        #         return reading_task_data[task_type][difficulty_level][task_id][data_type]
        if task_id in reading_task_data[task_type].keys():
            return reading_task_data[task_type][task_id][data_type]


def get_all_scores(statedb):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    all_scores = []
    for task_type in reading_task_data.keys():
        for task_id in reading_task_data[task_type]:
            score = get_reading_task_data_value(statedb, task_id, TaskDataKeys.SCORE)
            if score is not None:
                all_scores.append(score)
    return all_scores


def set_reading_task_value(statedb, task_id, data_type, value):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    for task_type in reading_task_data:
        # for difficulty_level in reading_task_data[task_type]:
        #     if task_id in reading_task_data[task_type][difficulty_level].keys():
        #         reading_task_data[task_type][difficulty_level][task_id][TaskDataKeys.SCORE] = score
        if task_id in reading_task_data[task_type].keys():
            reading_task_data[task_type][task_id][data_type] = value
            print(f"Reading task '{task_id}' {data_type} set to {value}")
            print(f"Reading task '{task_id}' score set to {value}")
    save_to_database(statedb, reading_task_data)


def save_to_database(statedb, reading_task_data):
    statedb.set(DatabaseKeys.READING_TASK_DATA, reading_task_data)
