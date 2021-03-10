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
        "first interaction datetime": None,
        "good to chat": None,
        "is done evaluation today": False,
        "is interaction finished": True,
        "is off checkin": False,
        "is prompted by user": False,
        "is run prompted content": False,
        "last interaction datetime": None,
        "last update datetime": None,
        "next checkin datetime": datetime.datetime(2021, 2, 6, 1, 30),
        "number of prompted today": 0,
        "reading performance": {},
        "user name": None,
    }

TIME_WINDOW_FOR_CHECKIN_KEY = "time window for checkin"
TIME_WINDOW_FOR_CHECKIN_VALUE = 15

PARAM_TEST_KEYS = (
    TIME_WINDOW_FOR_CHECKIN_KEY,
)
PARAM_TEST_VALUES = (
    TIME_WINDOW_FOR_CHECKIN_VALUE,
)
PARAM_PAIRS = zip(PARAM_TEST_KEYS, PARAM_TEST_VALUES)


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
