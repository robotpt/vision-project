#!/usr/bin/python3.8
import datetime
import freezegun
import mock
import pytest

from controllers import VisionProjectDelegator


@pytest.fixture
def vision_project_delegator(statedb, paramdb):
    return VisionProjectDelegator(
        statedb,
        paramdb,
        is_run_demo_interaction=False
    )


def test_determine_first_interaction(vision_project_delegator, statedb):
    assert vision_project_delegator._is_first_interaction()
    statedb.set("first interaction time", datetime.datetime(2021, 1, 15, 6, 0, 0))
    assert not vision_project_delegator._is_first_interaction()


def test_determine_scheduled_interaction(vision_project_delegator, statedb):
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin time", next_checkin_time)

    invalid_checkin_times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:15",
        "2021-1-12 12:00:16"
    ]

    for invalid_time in invalid_checkin_times:
        with freezegun.freeze_time(invalid_time):
            assert not vision_project_delegator._is_time_for_scheduled_interaction()

    valid_checkin_times = [
        "2021-1-12 11:59:46",
        "2021-1-12 12:00:00",
        "2021-1-12 12:00:14"
    ]

    for valid_time in valid_checkin_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator._is_time_for_scheduled_interaction()
