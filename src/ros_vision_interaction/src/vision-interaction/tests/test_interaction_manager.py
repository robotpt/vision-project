#!/usr/bin/python3.8
import freezegun
import pytest

from controllers import InteractionManager
from vision_project_tools.vision_engine import VisionInteractionEngine as InteractionEngine


def mock_run(_, planner):

    while planner.is_active:
        while planner.is_active:
            for node_name in mock_run_next_plan(planner):
                yield node_name


def mock_run_next_plan(planner):
    message_name, pre_hook, post_hook = planner.pop_plan(is_return_hooks=True)
    yield message_name

    pre_hook()
    post_hook()


@pytest.fixture
def interaction_manager(statedb, interaction_builder, monkeypatch):
    monkeypatch.setattr(InteractionEngine, "modified_run", mock_run)
    manager = InteractionManager(
        statedb=statedb,
        interaction_builder=interaction_builder
    )
    return manager


def test_run_first_interaction(interaction_manager, statedb):
    assert not interaction_manager._state_database.is_set("first interaction")
    interaction_manager.run_interaction_once("first interaction")
    assert interaction_manager._state_database.is_set("first interaction datetime")


def test_run_scheduled_interaction(interaction_manager, statedb):
    with freezegun.freeze_time("2021-02-10"):
        assert not statedb.is_set("last interaction time")
        interaction_manager.run_interaction_once("scheduled interaction")
        assert statedb.is_set("is done eval today")
