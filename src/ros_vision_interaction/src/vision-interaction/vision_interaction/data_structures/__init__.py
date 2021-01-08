#!/usr/bin/env python3.8

import logging as _logging
import pymongo as _pymongo
import os as _os
import rospy as _rospy

from interaction_engine.text_populator import TextPopulator as _TextPopulator
from interaction_engine.text_populator import DatabasePopulator as _DatabasePopulator
from interaction_engine.text_populator import VarietyPopulator as _VarietyPopulator
from mongodb_statedb import StateDb as _StateDb


_resources_directory = _rospy.get_param('vision-project/resources/path/deployment')
_variation_file = _os.path.join(_resources_directory, 'variations.json')

# set up database
_host = _rospy.get_param(
    "mongodb/host",
    "localhost"
)
_port = _rospy.get_param(
    "mongodb/port",
    62345
)
state_database = _StateDb(
    _pymongo.MongoClient(_host, _port)
)

_keys = [
    "user name",
    "time for next interaction",
    "first interaction time"
]
for key in _keys:
    try:
        state_database.create(key, None)
    except KeyError:
        _logging.info("{} already exists".format(key))

# set up populators
_database_populator = _DatabasePopulator(state_database)
_variety_populator = _VarietyPopulator(_variation_file)
text_populator = _TextPopulator(_variety_populator, _database_populator)
