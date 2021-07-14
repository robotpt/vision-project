import json
import logging

from vision_project_tools.constants import DatabaseKeys
from vision_project_tools.engine_statedb import EngineStateDb

logging.basicConfig(level=logging.INFO)


class Tasks:
    IREST = "IReST"
    MNREAD = "MNread"
    SELF_SELECTED = "self selected"
    SPOT_READING = "spot reading"
    SRT = "SRT"


class ReadingTaskDatabase:

    def __init__(
            self,
            statedb,
            reading_task_data
    ):
        self._state_database = statedb
        self._check_reading_task_dict(reading_task_data)
        self._reading_task_data = reading_task_data

    def _check_reading_task_dict(self, task_dict):
        if type(task_dict) is not dict:
            raise TypeError("Reading task data must be a dictionary")
        if set(task_dict.keys()) != {Tasks.IREST, Tasks.MNREAD, Tasks.SELF_SELECTED, Tasks.SPOT_READING, Tasks.SRT}:
            raise KeyError("")
