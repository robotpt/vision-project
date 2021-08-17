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


class TaskDataKeys:
    ANSWER = "answer"
    COLOR = "color"
    CORRECT = "correct"
    SCORE = "score"
    WORD_COUNT = "word_count"


def get_new_day_reading_task(statedb):
    task_type = get_current_reading_task_type()
    # current_difficulty_level = statedb.get(DatabaseKeys.DIFFICULTY_LEVEL)
    return get_random_reading_task_id(statedb, task_type)


def get_current_reading_task_type():
    current_weekday = datetime.datetime.now().weekday()
    if current_weekday == 0:  # Monday
        task_type = Tasks.SPOT_READING
    elif current_weekday == 1:  # Tuesday
        task_type = Tasks.SRT
    elif current_weekday == 2:  # Wednesday
        task_type = Tasks.SPOT_READING
    elif current_weekday == 3:  # Thursday
        task_type = Tasks.IREST
    elif current_weekday == 4:  # Friday
        task_type = Tasks.SPOT_READING
    elif current_weekday == 5:  # Saturday
        task_type = Tasks.SRT
    elif current_weekday == 6:  # Sunday
        task_type = Tasks.MNREAD
    else:
        logging.info(f"No tasks for weekday index {current_weekday}, setting task type to 'spot reading'.")
        task_type = Tasks.SPOT_READING
    return task_type


def get_random_reading_task_id(statedb, task_type, difficulty_level=None):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    # tasks = reading_task_data[task_type][difficulty_level]
    tasks = reading_task_data[task_type]
    # logging.info(f"Possible tasks: {tasks}")
    possible_tasks = []
    for task in tasks.keys():
        if tasks[task][TaskDataKeys.SCORE] is None:
            possible_tasks.append(task)
    return random.choice(possible_tasks)


def get_reading_task_data_value(statedb, task_id, data_type):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    if data_type not in [
        TaskDataKeys.ANSWER,
        TaskDataKeys.COLOR,
        TaskDataKeys.CORRECT,
        TaskDataKeys.SCORE,
        TaskDataKeys.WORD_COUNT
    ]:
        raise KeyError(f"'{data_type}' is not a valid reading task data type.")
    for task_type in reading_task_data:
        # for difficulty_level in reading_task_data[task_type]:
        #     if task_id in reading_task_data[task_type][difficulty_level].keys():
        #         return reading_task_data[task_type][difficulty_level][task_id][data_type]
        if task_id in reading_task_data[task_type].keys():
            return reading_task_data[task_type][task_id][data_type]


def set_reading_task_score(statedb, task_id, score):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    for task_type in reading_task_data:
        # for difficulty_level in reading_task_data[task_type]:
        #     if task_id in reading_task_data[task_type][difficulty_level].keys():
        #         reading_task_data[task_type][difficulty_level][task_id][TaskDataKeys.SCORE] = score
        if task_id in reading_task_data[task_type].keys():
            reading_task_data[task_type][task_id][TaskDataKeys.SCORE] = score
            logging.info(f"Reading task '{task_id}' score set to {score}")
    save_to_database(statedb, reading_task_data)


def save_to_database(statedb, reading_task_data):
    statedb.set(DatabaseKeys.READING_TASK_DATA, reading_task_data)
