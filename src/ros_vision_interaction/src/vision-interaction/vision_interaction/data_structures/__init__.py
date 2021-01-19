#!/usr/bin/env python3.8

import datetime as _datetime
import logging as _logging
import pymongo as _pymongo
import os as _os
# import rospy as _rospy

from engine_statedb import EngineStateDb as StateDb
from interaction_engine.text_populator import TextPopulator as _TextPopulator
from interaction_engine.text_populator import DatabasePopulator as _DatabasePopulator
from interaction_engine.text_populator import VarietyPopulator as _VarietyPopulator

# _demo_resources_directory = _rospy.get_param('vision-project/resources/path/demo')
demo_resources_directory = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources/demo-interaction/'
_demo_variation_file = _os.path.join(demo_resources_directory, 'variations.json')

# _deployment_resources_directory = _rospy.get_param('vision-project/resources/path/deployment')
deployment_resources_directory = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources/deployment/'
_deployment_variation_file = _os.path.join(deployment_resources_directory, 'variations.json')

DATABASE_NAME = "vision-project"


def _init_db(db, key_values):
    for key in key_values:
        try:
            db.create(key, key_values[key])
        except KeyError:
            _logging.info("{} already exists".format(key))


# set up state database
_host = "localhost"
_port = 62345
state_database = StateDb(
    _pymongo.MongoClient(_host, _port),
    database_name=DATABASE_NAME,
    collection_name="state_db"
)

_state_db_key_values = {
    "first interaction time": None,
    "last interaction time": None,
    "next checkin time": None,
    "user name": None
}

_init_db(state_database, _state_db_key_values)

# set up parameter database
param_database = StateDb(
    _pymongo.MongoClient(_host, _port),
    database_name=DATABASE_NAME,
    collection_name="param_db"
)

_param_db_keys = {
    "minutes between demo interactions": 5,
    "time window for checkin": 15
}

_init_db(param_database, _param_db_keys)

# set up populators
_database_populator = _DatabasePopulator(state_database)
_demo_variety_populator = _VarietyPopulator(_demo_variation_file)
_deployment_variety_populator = _VarietyPopulator(_deployment_variation_file)

demo_text_populator = _TextPopulator(_demo_variety_populator, _database_populator)
deployment_text_populator = _TextPopulator(_deployment_variety_populator, _database_populator)
