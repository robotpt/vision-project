import logging

from vision_project_tools.constants import DatabaseKeys, INITIAL_STATE_DB, Interactions
from vision_project_tools.engine_statedb import EngineStateDb
from vision_project_tools.vision_engine import VisionInteractionEngine

logging.basicConfig(level=logging.INFO)


def init_db(db, key_values):
    for key in key_values:
        try:
            db.create(key, key_values[key])
        except KeyError:
            logging.info("{} already exists".format(key))
