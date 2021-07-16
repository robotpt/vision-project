import datetime
import json
import logging
import random

from vision_project_tools.constants import DatabaseKeys
from vision_project_tools.engine_statedb import EngineStateDb

logging.basicConfig(level=logging.INFO)


class Tasks:
    IREST = "IReST"
    MNREAD = "MNread"
    SELF_SELECTED = "self selected"
    SKREAD = "SKread"
    SPOT_READING = "spot reading"
    SRT = "SRT"


class ReadingTaskManager:

    def __init__(
            self,
            statedb,
            reading_task_data_dict
    ):
        self._state_database = statedb
        self._check_reading_task_dict(reading_task_data_dict)
        self._reading_task_data = reading_task_data_dict

    def _check_reading_task_dict(self, task_dict):
        if type(task_dict) is not dict:
            raise TypeError("Reading task data must be a dictionary")
        if task_dict.keys() != {Tasks.IREST, Tasks.MNREAD, Tasks.SELF_SELECTED, Tasks.SPOT_READING, Tasks.SRT}:
            raise KeyError("")

    def get_current_reading_task(self):
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
        current_difficulty_level = self._state_database.get(DatabaseKeys.DIFFICULTY_LEVEL)
        task_id = self.get_reading_task_id(task_type, current_difficulty_level)
        self._state_database.set(DatabaseKeys.CURRENT_READING_ID, task_id)

    def get_reading_task_id(self, task_type, difficulty_level):
        possible_tasks = []
        for task_id in self._reading_task_data[task_type][difficulty_level]:
            if task_id["score"] is None:
                possible_tasks.append(task_id)
        return random.choice(possible_tasks)

    def get_word_count(self, task_id):
        for task_type in self._reading_task_data:
            for difficulty_level in task_type:
                if task_id in difficulty_level.keys():
                    return difficulty_level[task_id]["word_count"]

    def save_to_database(self):
        self._state_database.set(DatabaseKeys.READING_TASK_DATA, self._reading_task_data)
