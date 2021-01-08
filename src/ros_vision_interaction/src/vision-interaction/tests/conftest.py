# conftest.py

import datetime
import mongomock
import pytest

from mongodb_statedb import StateDb

DATABASE_NAME = 'vision_project'
COLLECTION_NAME = 'state'

NAME_KEY = 'user name'
NAME_VALUE = 'Emily Zhou'
NEXT_INTERACTION_TIME_KEY = 'time for next scheduled interaction'
NEXT_INTERACTION_TIME_VALUE = datetime.datetime(2021, 1, 6)
LAST_CHECKIN_TIME_KEY = 'last checkin time'
LAST_CHECKIN_TIME_VALUE = datetime.datetime(2021, 1, 6)

TEST_KEYS = (
    NAME_KEY,
    NEXT_INTERACTION_TIME_KEY,
    LAST_CHECKIN_TIME_KEY
)
TEST_VALUES = (
    NAME_VALUE,
    NEXT_INTERACTION_TIME_VALUE,
    LAST_CHECKIN_TIME_VALUE
)

STATE_PAIRS = zip(TEST_KEYS, TEST_VALUES)


@pytest.fixture
def mongo_client():
    client = mongomock.MongoClient()
    db = client[DATABASE_NAME][COLLECTION_NAME]
    for pair in STATE_PAIRS:
        db.insert_one({pair[0]: pair[1]})
    return client


@pytest.fixture
def statedb(mongo_client):
    return StateDb(mongo_client)
