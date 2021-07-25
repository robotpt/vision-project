#!/usr/bin/python3.8
import json
import mock
import mongomock
import os
import pytest

from interaction_builder import InteractionBuilder
from vision_project_tools.constants import INITIAL_STATE_DB
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

DATABASE_NAME = 'vision_project'
STATE_COLLECTION_NAME = 'state'
PARAM_COLLECTION_NAME = 'params'


@pytest.fixture
def state_mongo_client():
    client = mongomock.MongoClient()
    db = client[DATABASE_NAME][STATE_COLLECTION_NAME]
    for key in INITIAL_STATE_DB:
        db.insert_one(
            {'_id': key, 'value': INITIAL_STATE_DB[key]},
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
def deployment_interaction_dict():
    deployment_interaction_dict = {}
    cwd = os.path.dirname(os.path.abspath(__file__))
    resources_directory = os.path.join(cwd, '..', '..', '..', 'resources')

    interaction_files = [
        os.path.join(resources_directory, 'deployment', 'first_interaction.json'),
        os.path.join(resources_directory, 'deployment', 'scheduled.json'),
        os.path.join(resources_directory, 'deployment', 'prompted.json')
    ]
    for file in interaction_files:
        with open(file) as f:
            deployment_interaction_dict.update(json.load(f))
    return deployment_interaction_dict


@pytest.fixture
def interaction_builder(statedb, deployment_interaction_dict):
    cwd = os.path.dirname(os.path.abspath(__file__))
    resources_directory = os.path.join(cwd, '..', '..', '..', 'resources')
    interaction_variations_file = os.path.join(resources_directory, 'deployment', 'interaction_variations.json')
    grit_dialogue_variations_file = os.path.join(resources_directory, 'deployment', 'grit_dialogue.json')
    scheduled_variations_file = os.path.join(resources_directory, 'deployment', 'scheduled_variations.json')

    builder = InteractionBuilder(
        interaction_dict=deployment_interaction_dict,
        variations_files=[
            interaction_variations_file,
            grit_dialogue_variations_file,
            scheduled_variations_file
        ],
        statedb=statedb
    )
    return builder
