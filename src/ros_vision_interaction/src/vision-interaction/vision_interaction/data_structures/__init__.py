#!/usr/bin/env python3.8

import logging as _logging
import pymongo as _pymongo
import os as _os
import rospy as _rospy

from data_structures.engine_statedb import EngineStateDb as StateDb
from interaction_engine.text_populator import TextPopulator as _TextPopulator
from interaction_engine.text_populator import DatabasePopulator as _DatabasePopulator
from interaction_engine.text_populator import VarietyPopulator as _VarietyPopulator

_demo_resources_directory = _rospy.get_param('vision-project/resources/path/demo')
_demo_variation_file = _os.path.join(_demo_resources_directory, 'variations.json')

_deployment_resources_directory = _rospy.get_param('vision-project/resources/path/deployment')
_deployment_variation_file = _os.path.join(_deployment_resources_directory, 'variations.json')

# set up database
_host = _rospy.get_param(
    "mongodb/host",
    "localhost"
)
_port = _rospy.get_param(
    "mongodb/port",
    62345
)
state_database = StateDb(
    _pymongo.MongoClient(_host, _port)
)

_keys = [
    "first interaction time",
    "last interaction time",
    "next checkin time",
    "user name",
]
for key in _keys:
    try:
        state_database.create(key, None)
    except KeyError:
        _logging.info("{} already exists".format(key))

# set up populators
_database_populator = _DatabasePopulator(state_database)
_demo_variety_populator = _VarietyPopulator(_demo_variation_file)
_deployment_variety_populator = _VarietyPopulator(_deployment_variation_file)

demo_text_populator = _TextPopulator(_demo_variety_populator, _database_populator)
deployment_text_populator = _TextPopulator(_deployment_variety_populator, _database_populator)
