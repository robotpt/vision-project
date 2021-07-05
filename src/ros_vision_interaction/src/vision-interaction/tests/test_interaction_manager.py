#!/usr/bin/python3.8
import datetime
import freezegun
import pytest

from controllers import InteractionManager
from vision_project_tools.constants import DatabaseKeys, Interactions
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
    assert not interaction_manager._state_database.is_set(DatabaseKeys.FIRST_INTERACTION_DATETIME)
    interaction_manager.run_interaction_once(Interactions.FIRST_INTERACTION)
    assert interaction_manager._state_database.is_set(DatabaseKeys.FIRST_INTERACTION_DATETIME)


def test_run_scheduled_interaction(interaction_manager, statedb):
    with freezegun.freeze_time("2021-02-10"):
        assert not statedb.is_set(DatabaseKeys.LAST_INTERACTION_DATETIME)
        interaction_manager.run_interaction_once(Interactions.SCHEDULED_INTERACTION)
        assert statedb.is_set(DatabaseKeys.IS_DONE_EVAL_TODAY)


def test_run_reading_evaluation(interaction_manager, statedb):
    current_eval_index = statedb.get(DatabaseKeys.READING_EVAL_INDEX)
    statedb.set(DatabaseKeys.GOOD_TO_CHAT, "Yes")
    statedb.set(DatabaseKeys.IS_DO_EVALUATION, "Yes")
    interaction_manager.run_interaction_once(Interactions.SCHEDULED_INTERACTION)
    assert statedb.is_set(DatabaseKeys.IS_DONE_EVAL_TODAY)
    assert statedb.get(DatabaseKeys.IS_INTERACTION_FINISHED)
    assert statedb.get(DatabaseKeys.READING_EVAL_INDEX) == current_eval_index + 1


def test_run_prompted_interaction(interaction_manager, statedb):
    with freezegun.freeze_time("2021-02-10"):
        statedb.set(DatabaseKeys.GOOD_TO_CHAT, "Yes")
        assert statedb.get(DatabaseKeys.NUM_OF_PROMPTED_TODAY) == 0
        interaction_manager.run_interaction_once(Interactions.PROMPTED_INTERACTION)
        assert statedb.get(DatabaseKeys.NUM_OF_PROMPTED_TODAY) == 1


def test_determine_is_do_goal_setting(interaction_manager, statedb):
    # less than a week after first interaction
    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, first_interaction_datetime)
    statedb.set(DatabaseKeys.FEELINGS_INDEX, 2)
    statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT, 3)
    statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE, 3)
    with freezegun.freeze_time("2021-04-05"):
        assert not interaction_manager._is_do_goal_setting()

    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, first_interaction_datetime)
    statedb.set(DatabaseKeys.FEELINGS_INDEX, 2)
    statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT, 3)
    statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE, 3)
    statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING, 6)
    statedb.set(DatabaseKeys.LAST_5_EVAL_SCORES, [5, 5, 5, 5, 5])
    statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 4)
    with freezegun.freeze_time("2021-04-10"):
        assert not interaction_manager._is_do_goal_setting()
        statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING, 7)
        assert interaction_manager._is_do_goal_setting()


def test_determine_is_do_mindfulness(interaction_manager, statedb):
    # less than a week after first interaction
    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, first_interaction_datetime)
    with freezegun.freeze_time("2021-04-05"):
        assert not interaction_manager._is_do_mindfulness()

    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, first_interaction_datetime)
    statedb.set(DatabaseKeys.FEELINGS_INDEX, 2)
    statedb.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS, 2)
    statedb.set(DatabaseKeys.LAST_5_EVAL_SCORES, [5, 5, 5, 5, 5])
    statedb.set(DatabaseKeys.CURRENT_EVAL_SCORE, 4)
    with freezegun.freeze_time("2021-04-10"):
        assert interaction_manager._is_do_mindfulness()
