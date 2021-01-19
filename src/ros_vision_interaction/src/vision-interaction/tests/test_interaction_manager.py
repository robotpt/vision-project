#!/usr/bin/python3.8
import datetime
import mock
import pytest

from controllers import InteractionManager


@pytest.fixture
def interaction_manager(statedb, paramdb):
    with mock.patch('builtins.open') as mock_open:
        mock_open.side_effect = mock.mock_open(read_data="{}")
        manager = InteractionManager(
            statedb=statedb,
            paramdb=paramdb
        )
    return manager
