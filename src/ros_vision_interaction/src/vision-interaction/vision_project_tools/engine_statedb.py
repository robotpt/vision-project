#!/usr/bin/python3.8
from mongodb_statedb import StateDb


class EngineStateDb(StateDb):

    def is_set(self, key):
        return self.exists(key) and self.get(key) is not None
