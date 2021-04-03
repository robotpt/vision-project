#!/usr/bin/python3.8
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
    deployment_interaction_dict = {
        "greeting": {
            "nodes": {
                "greeting": {
                    "transitions": ["exit"],
                    "content": "Hello there!",
                    "options": ["Next"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "greeting"
        },
        "first checkin": {
            "nodes": {
                "first checkin": {
                    "transitions": ["exit"],
                    "content": "Nice to meet you, my name is QT!",
                    "options": ["Next"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "first checkin"
        },
        "scheduled checkin": {
            "nodes": {
                "scheduled checkin": {
                    "transitions": ["exit"],
                    "content": "This is a scheduled checkin.",
                    "options": ["exit"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "scheduled checkin"
        },
        "prompted checkin": {
            "nodes": {
                "prompted checkin": {
                    "transitions": ["exit"],
                    "content": "This is a prompted checkin.",
                    "options": ["exit"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "prompted checkin"
        },
        "schedule next checkin": {
            "nodes": {
                "schedule": {
                    "transitions": ["exit"],
                    "content": "When should we plan to talk tomorrow?",
                    "options": ["Tomorrow"],
                    "args": ["15", "12:00"],
                    "message_type": "time entry",
                    "result_convert_from_str_fn": "next day checkin time",
                    "result_db_key": "next checkin time"
                }
            },
            "start_node_name": "schedule"
        },
        "evaluation": {
            "nodes": {
                "evaluation": {
                    "transitions": ["exit"],
                    "content": "[reading evaluation]",
                    "options": ["Next"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "evaluation"
        },
        "ask to chat": {
            "nodes": {
                "ask to chat": {
                    "transitions": ["exit", "that's ok"],
                    "content": "Is this a good time to chat?",
                    "options": ["Yes", "No"],
                    "message_type": "multiple choice one column",
                    "result_db_key": "good to chat"
                },
                "that's ok": {
                    "transitions": ["exit"],
                    "content": "That's alright.",
                    "options": ["Next"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "ask to chat"
        },
        "talk about vision": {
            "nodes": {
                "talk about vision": {
                    "transitions": ["automated response to patient"],
                    "content": "Please tell me how you're feeling about your vision.",
                    "options": ["Next"],
                    "message_type": "multiple choice one column"
                },
                "automated response to patient": {
                    "transitions": ["exit"],
                    "content": "[automated response]",
                    "options": ["Next"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "talk about vision"
        },
        "goodbye": {
            "nodes": {
                "goodbye": {
                    "transitions": ["exit"],
                    "content": "Goodbye!",
                    "options": ["Bye!"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "goodbye"
        },
        "ask to do scheduled": {
            "nodes": {
                "ask to do scheduled": {
                    "transitions": ["exit"],
                    "content": "Would you like to do the reading evaluation now?",
                    "options": ["Yes", "No"],
                    "message_type": "multiple choice one column",
                    "result_db_key": "is off checkin"
                }
            },
            "start_node_name": "ask to do scheduled"
        },
        "too many checkins": {
            "nodes": {
                "too many checkins": {
                    "transitions": ["exit"],
                    "content": "> 3 checkins, talk more tomorrow!",
                    "options": ["Oops"],
                    "message_type": "multiple choice one column"
                }
            },
            "start_node_name": "too many checkins"
        }
    }
    with mock.patch('builtins.open', mock.mock_open(read_data="{}")) as mock_open:
        handlers = [mock_open.return_value, mock.mock_open(read_data="{}").return_value]
        mock_open.side_effect = handlers
        builder = InteractionBuilder(
            interaction_dict=deployment_interaction_dict,
            variations_files="interaction_variations.json",
            statedb=statedb
        )
    return builder
