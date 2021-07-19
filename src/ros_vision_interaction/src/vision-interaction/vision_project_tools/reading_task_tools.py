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
    COLOR = "color"
    SCORE = "score"
    WORD_COUNT = "word_count"


def set_new_day_reading_task(statedb):
    task_type = get_current_reading_task_type()
    current_difficulty_level = statedb.get(DatabaseKeys.DIFFICULTY_LEVEL)
    return get_random_reading_task_id(statedb, task_type, current_difficulty_level)


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


def get_random_reading_task_id(statedb, task_type, difficulty_level):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    possible_tasks = reading_task_data[task_type][difficulty_level]
    for task in possible_tasks.keys():
        if possible_tasks[task][TaskDataKeys.SCORE] is not None:
            possible_tasks.pop(task)
    return random.choice(list(possible_tasks.keys()))


def get_reading_task_data_value(statedb, task_id, data_type):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    if data_type not in [
        TaskDataKeys.COLOR,
        TaskDataKeys.SCORE,
        TaskDataKeys.WORD_COUNT
    ]:
        raise KeyError(f"'{data_type}' is not a valid reading task data type.")
    for task_type in reading_task_data:
        for difficulty_level in reading_task_data[task_type]:
            if task_id in reading_task_data[task_type][difficulty_level].keys():
                return reading_task_data[task_type][difficulty_level][task_id][data_type]


def set_reading_task_score(statedb, task_id, score):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    for task_type in reading_task_data:
        for difficulty_level in reading_task_data[task_type]:
            if task_id in reading_task_data[task_type][difficulty_level].keys():
                reading_task_data[task_type][difficulty_level][task_id][TaskDataKeys.SCORE] = score
                save_to_database(statedb)


def save_to_database(statedb):
    reading_task_data = statedb.get(DatabaseKeys.READING_TASK_DATA)
    statedb.set(DatabaseKeys.READING_TASK_DATA, reading_task_data)
