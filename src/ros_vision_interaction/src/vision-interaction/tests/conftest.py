#!/usr/bin/python3.8
import datetime
import mock
import mongomock
import pytest

from interaction_builder import InteractionBuilder
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

DATABASE_NAME = 'vision_project'
STATE_COLLECTION_NAME = 'state'
PARAM_COLLECTION_NAME = 'params'

NAME_KEY = 'user name'
FIRST_INTERACTION_TIME_KEY = 'first interaction time'
LAST_CHECKIN_TIME_KEY = 'last checkin time'
NEXT_CHECKIN_TIME_KEY = 'next checkin time'
NAME_VALUE = 'Emily Zhou'
FIRST_INTERACTION_TIME_VALUE = None
LAST_CHECKIN_TIME_VALUE = None
NEXT_CHECKIN_TIME_VALUE = None

STATE_TEST_KEYS = (
    NAME_KEY,
    FIRST_INTERACTION_TIME_KEY,
    LAST_CHECKIN_TIME_KEY,
    NEXT_CHECKIN_TIME_KEY
)
STATE_TEST_VALUES = (
    NAME_VALUE,
    FIRST_INTERACTION_TIME_VALUE,
    LAST_CHECKIN_TIME_VALUE,
    NEXT_CHECKIN_TIME_VALUE
)
STATE_PAIRS = zip(STATE_TEST_KEYS, STATE_TEST_VALUES)

TIME_BETWEEN_DEMO_INTERACTIONS_KEY = "time between demo interaction_builder"
TIME_WINDOW_FOR_CHECKIN_KEY = "time window for checkin"
TIME_BETWEEN_DEMO_INTERACTIONS_VALUE = 15
TIME_WINDOW_FOR_CHECKIN_VALUE = 15

PARAM_TEST_KEYS = (
    TIME_BETWEEN_DEMO_INTERACTIONS_KEY,
    TIME_WINDOW_FOR_CHECKIN_KEY,
)
PARAM_TEST_VALUES = (
    TIME_BETWEEN_DEMO_INTERACTIONS_VALUE,
    TIME_WINDOW_FOR_CHECKIN_VALUE,
)
PARAM_PAIRS = zip(PARAM_TEST_KEYS, PARAM_TEST_VALUES)


@pytest.fixture
def state_mongo_client():
    client = mongomock.MongoClient()
    db = client[DATABASE_NAME][STATE_COLLECTION_NAME]
    db.insert_many([
        {'_id': NAME_KEY, 'value': NAME_VALUE},
        {'_id': FIRST_INTERACTION_TIME_KEY, 'value': FIRST_INTERACTION_TIME_VALUE},
        {'_id': LAST_CHECKIN_TIME_KEY, 'value': LAST_CHECKIN_TIME_VALUE},
        {'_id': NEXT_CHECKIN_TIME_KEY, 'value': NEXT_CHECKIN_TIME_VALUE},
    ])
    return client


@pytest.fixture
def param_mongo_client():
    client = mongomock.MongoClient()
    db = client[DATABASE_NAME][PARAM_COLLECTION_NAME]
    db.insert_many([
        {'_id': TIME_BETWEEN_DEMO_INTERACTIONS_KEY, 'value': TIME_BETWEEN_DEMO_INTERACTIONS_VALUE},
        {'_id': TIME_WINDOW_FOR_CHECKIN_KEY, 'value': TIME_WINDOW_FOR_CHECKIN_VALUE}
    ])
    return client


@pytest.fixture
def statedb(state_mongo_client):
    return StateDb(
        state_mongo_client,
        database_name=DATABASE_NAME,
        collection_name=STATE_COLLECTION_NAME
    )


@pytest.fixture
def paramdb(param_mongo_client):
    return StateDb(
        param_mongo_client,
        database_name=DATABASE_NAME,
        collection_name=PARAM_COLLECTION_NAME
    )


@pytest.fixture
def interaction_builder(statedb):
    demo_interaction_dict = {
        "demo interaction": {
            "nodes": {
                "demo": {
                    "content": "demo",
                    "message_type": "multiple choice",
                    "transitions": "exit",
                    "options": "Next"
                }
            },
            "start_node_name": "demo"
        }
    }
    deployment_interaction_dict = {
        "first interaction": {
            "nodes": {
                "first": {
                    "content": "first interaction",
                    "message_type": "multiple choice",
                    "transitions": "exit",
                    "options": "Next"
                }
            },
            "start_node_name": "first"
        },
        "greeting": {
            "nodes": {
                "greeting": {
                    "content": "greeting",
                    "message_type": "multiple choice",
                    "transitions": "exit",
                    "options": "Next"
                }
            },
            "start_node_name": "greeting"
        },
        "introduce QT": {
            "nodes": {
                "introduce QT": {
                    "content": "introduce QT",
                    "message_type": "multiple choice",
                    "transitions": "exit",
                    "options": "Next"
                }
            },
            "start_node_name": "introduce QT"
        },
        "prompted interaction": {
            "nodes": {
                "prompted interaction": {
                    "content": "prompted interaction",
                    "message_type": "multiple choice",
                    "transitions": "exit",
                    "options": "Next"
                }
            },
            "start_node_name": "prompted interaction"
        },
        "reading evaluation": {
            "nodes": {
                "reading evaluation": {
                    "content": "reading evaluation",
                    "message_type": "multiple choice",
                    "transitions": "exit",
                    "options": "Next"
                }
            },
            "start_node_name": "reading evaluation"
        },
        "scheduled interaction": {
            "nodes": {
                "scheduled interaction": {
                    "content": "scheduled interaction",
                    "message_type": "multiple choice",
                    "transitions": "exit",
                    "options": "Next"
                }
            },
            "start_node_name": "scheduled interaction"
        }
    }
    with mock.patch('builtins.open', mock.mock_open(read_data="{}")) as mock_open:
        handlers = [mock_open.return_value, mock.mock_open(read_data="{}").return_value]
        mock_open.side_effect = handlers
        builder = InteractionBuilder(
            demo_interaction_dict=demo_interaction_dict,
            demo_variations_file="variations.json",
            deployment_interaction_dict=deployment_interaction_dict,
            deployment_variations_file="variations.json",
            statedb=statedb
        )
    return builder
