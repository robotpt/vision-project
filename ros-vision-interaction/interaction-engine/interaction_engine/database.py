#!/usr/bin/env python
import json
import logging

from interaction_engine.utils import get_database_from_file

logging.basicConfig(level=logging.INFO)


class Database(object):

    def __init__(self, database_file, default_database_keys=None):

        self._database_file = self._make_sure_database_file_is_valid(database_file)
        self._check_default_database_keys(default_database_keys)
        self._database = self._set_initial_db(default_database_keys)

    def _make_sure_database_file_is_valid(self, database_file):
        if database_file is not None:
            if type(database_file) is not str:
                raise TypeError("Not a valid file name.")
            if not database_file.endswith(".json"):
                raise TypeError("Must be a .json file.")
        return database_file

    def _check_default_database_keys(self, default_database_keys):
        if default_database_keys is not None:
            if type(default_database_keys) is not list:
                raise TypeError("Database keys must be a list.")
            for key in default_database_keys:
                if type(key) is not str:
                    raise TypeError("Database key must be a string.")

    def _set_initial_db(self, default_database_keys):
        database = get_database_from_file(self._database_file)
        if not database:
            logging.info("Database in file is empty.")
            if default_database_keys is not None:
                return {key: "" for key in default_database_keys}
            else:
                return {}
        else:
            return database

    def __getitem__(self, item):
        return self._database[item]

    def __setitem__(self, key, value):
        if type(key) is not str:
            raise TypeError("Key must be a string.")
        if type(value) is not str:
            raise TypeError("Values must be a string.")
        self._database[key] = value
        self.save_to_database_file()

    def save_to_database_file(self):
        with open(self._database_file, "w") as f:
            json.dump(self._database, f)

    def clear_database_value(self, key):
        self._database[key] = ""

    def clear_entire_database(self):
        for key in self._database:
            self.clear_database_value(key)

    @property
    def database(self):
        return self._database

    @database.setter
    def database(self, database):
        if type(database) is not dict:
            raise TypeError("Database must be a dict.")
        self._database = database

    @property
    def database_file(self):
        return self._database_file

