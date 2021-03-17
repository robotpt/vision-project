#!/usr/bin/python3.8
import datetime
import json
import mock
import mongomock
import pytest

from interaction_builder import InteractionBuilder
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

DATABASE_NAME = 'vision_project'
STATE_COLLECTION_NAME = 'state'
PARAM_COLLECTION_NAME = 'params'

STATE_DB_KEYS_AND_VALUES = {
        "average eval score": None,
        "current eval score": None,
        "first interaction datetime": None,
        "good time to talk": False,
        "is done eval today": False,
        "is done prompted today": False,
        "is done perseverance today": False,
        "is done mindfulness today": False,
        "is done goal setting today": False,
        "is interaction finished": False,
        "is prompted by user": False,
        "is run prompted content": False,
        "last eval score": None,
        "last interaction datetime": None,
        "last update datetime": None,
        "next checkin datetime": None,
        "num of days since last eval": 0,
        "num of days since last prompt": 0,
        "num of days since last perseverance": 0,
        "num of days since last mindfulness": 0,
        "num of days since last goal setting": 0,
    }


@pytest.fixture
def state_mongo_client():
    client = mongomock.MongoClient()
    db = client[DATABASE_NAME][STATE_COLLECTION_NAME]
    for key in STATE_DB_KEYS_AND_VALUES:
        db.insert_one(
            {'_id': key, 'value': STATE_DB_KEYS_AND_VALUES[key]},
        )
    return client


@pytest.fixture
def statedb(state_mongo_client):
    return StateDb(
        state_mongo_client,
        database_name=DATABASE_NAME,
        collection_name=STATE_COLLECTION_NAME
    )


@pytest.fixture
def interaction_builder(statedb):
    interaction_file = '/root/catkin_ws/src/vision-project/src/ros_vision_interaction/resources/deployment/test_interactions.json'
    with open(interaction_file) as f:
        deployment_interaction_dict = json.load(f)
    with mock.patch('builtins.open', mock.mock_open(read_data="{}")) as mock_open:
        handlers = [mock_open.return_value, mock.mock_open(read_data="{}").return_value]
        mock_open.side_effect = handlers
        builder = InteractionBuilder(
            interaction_dict=deployment_interaction_dict,
            variations_files="interaction_variations.json",
            statedb=statedb
        )
    return builder
