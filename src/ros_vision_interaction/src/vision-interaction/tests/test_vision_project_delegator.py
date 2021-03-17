#!/usr/bin/python3.8
import datetime
import freezegun
import pytest

from controllers import VisionProjectDelegator


@pytest.fixture
def vision_project_delegator(statedb):
    return VisionProjectDelegator(
        statedb,
        update_window_seconds=15,
        minutes_between_interactions=1
    )


def test_determine_first_interaction(vision_project_delegator, statedb):
    assert vision_project_delegator._is_first_interaction()
    assert not vision_project_delegator._is_time_for_scheduled_interaction()
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 15, 6, 0, 0))
    assert not vision_project_delegator._is_first_interaction()


def test_determine_scheduled_interaction_not_prompted(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("last interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("is prompted by user", False)
    statedb.set("is run prompted content", False)
    statedb.set("is done eval today", False)
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)

    scheduled_window_times = [
        "2021-1-12 11:59:46",
        "2021-1-12 12:00:00",
        "2021-1-12 12:00:14",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:15",
    ]
    # check that scheduled is called when in time window
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator._is_time_for_scheduled_interaction()
            assert vision_project_delegator.get_interaction_type() == "scheduled interaction"

    outside_scheduled_window_times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 12:00:16"
    ]
    # check scheduled is not called when not in time window
    for invalid_time in outside_scheduled_window_times:
        with freezegun.freeze_time(invalid_time):
            assert not vision_project_delegator._is_time_for_scheduled_interaction()


def test_determine_interaction_prompted_no_evaluation(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("last interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)
    statedb.set("is prompted by user", True)
    statedb.set("is done eval today", False)
    statedb.set("is run prompted content", False)
    # check scheduled interaction is called when within scheduled interaction window and evaluation not done yet
    # window of +/- 15 minutes
    scheduled_window_times = [
        "2021-1-12 11:45:00",
        "2021-1-12 11:46:00",
        "2021-1-12 12:00:00",
        "2021-1-12 12:14:00",
        "2021-1-12 12:15:00",
    ]
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator.get_interaction_type() == "scheduled interaction"
    # check 'ask to run prompted' called when not within scheduled interaction window and evaluation not done yet
    scheduled_window_times = [
        "2021-1-12 11:44:59",
        "2021-1-12 12:15:01",
        "2021-1-12 11:00:00",
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
    ]
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator.get_interaction_type() == "ask to do evaluation"


def test_determine_interaction_prompted_evaluation_done(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("last interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)
    statedb.set("is prompted by user", True)
    statedb.set("is done eval today", True)
    statedb.set("is run prompted content", False)
    # check that prompted interaction is called when evaluation has been done and QT is prompted
    times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:00",
        "2021-1-12 12:00:15",
        "2021-1-12 12:00:16"
    ]
    for time in times:
        with freezegun.freeze_time(time):
            assert vision_project_delegator.get_interaction_type() == "prompted interaction"


def test_determine_off_checkin(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("last interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("is prompted by user", True)
    statedb.set("is run prompted content", False)
    statedb.set("is done eval today", False)
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)
    # check evaluation content is run after QT is prompted and participant agrees to do evaluation
    outside_scheduled_window_times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:15",
        "2021-1-12 12:00:16"
    ]
    for time in outside_scheduled_window_times:
        with freezegun.freeze_time(time):
            assert vision_project_delegator._is_off_checkin()


def test_is_done_reading_evaluation(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 2, 9, 6, 0, 0))
    statedb.set("next checkin datetime", datetime.datetime(2021, 2, 11, 2, 0, 0))
    statedb.set("last update datetime", datetime.datetime(2021, 2, 10, 12, 0, 0))
    statedb.set("num of days since last eval", 0)

    # check scheduled interaction is called when valid time and evaluation not done yet
    with freezegun.freeze_time("2021-02-11 2:00:00"):
        assert statedb.get("last update datetime") == datetime.datetime(2021, 2, 10, 12, 0, 0)
        vision_project_delegator.update()
        assert not statedb.get("is done eval today")
        assert vision_project_delegator.get_interaction_type() == "scheduled interaction"

    # check no interaction is called when valid time but evaluation already done
        with freezegun.freeze_time("2021-02-11 2:00:00"):
            statedb.set("is done eval today", True)
            assert vision_project_delegator.get_interaction_type() is None
