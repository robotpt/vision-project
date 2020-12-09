import json
import logging
import os
import pprint

logging.basicConfig(level=logging.INFO)


class Database:

    def __init__(
            self,
            database_file_name
    ):

        self._database_file = self._make_sure_database_file_is_valid(
            database_file_name
        )

        if not os.path.exists(database_file_name):
            self._database = {}
            self._create_new_database_file(self._database_file)

        self._database = self._load_database_from_file()

    def _create_new_database_file(self, database_file_name):
        directory = os.path.dirname(database_file_name)
        if directory != "":
            os.makedirs(directory, exist_ok=True)
        with open(self._database_file, "w"):
            self.save_to_database_file()

    def _load_database_from_file(self):
        with open(self._database_file) as f:
            try:
                database = json.load(f)
            except json.JSONDecodeError:
                logging.info("Error decoding JSON file, defaulting to empty database")
                database = {}
        return database

    def _make_sure_database_file_is_valid(self, database_file_name):
        if type(database_file_name) is not str:
            raise TypeError("Not a valid file name.")

        if not database_file_name.endswith(".json"):
            raise TypeError("Must be a .json file.")

        return database_file_name

    def __contains__(self, item):
        self._database = self._load_database_from_file()
        return item in self._database

    def __repr__(self):
        return pprint.pformat(self._database)

    def __getitem__(self, item):
        self._database = self._load_database_from_file()
        return self._database[item]

    def __setitem__(self, key, value):
        self._database = self._load_database_from_file()
        possible_value_types = [list, str, int, float]
        if type(key) is not str:
            raise TypeError("Key must be a string.")

        if value is None:
            value = ""
        elif type(value) in possible_value_types:
            if type(value) is list:
                self._check_if_items_in_list_are_valid(value, [str, int, float])
        else:
            raise TypeError("Invalid value: {value}, can't have type {type}".format(value=value, type=type(value)))

        self._database[key] = value
        self.save_to_database_file()

    def _check_if_items_in_list_are_valid(self, list, possible_value_types):
        for item in list:
            if type(item) not in possible_value_types:
                raise TypeError("List contains invalid value: {item}, type {type}".format(
                    item=item,
                    type=type(item)
                ))

    def clear_value(self, key):
        self._database = self._load_database_from_file()
        self._database[key] = ""
        self.save_to_database_file()

    def clear_entire_database(self):
        for key in self._database:
            self.clear_value(key)

    def create_key(self, key):
        self._database = self._load_database_from_file()
        self._database[key] = ""
        self.save_to_database_file()

    def delete_database_file(self):
        os.remove(self._database_file)

    def delete_key(self, key):
        try:
            self._database.pop(key)
        except KeyError as e:
            logging.info(e)
            pass
        self.save_to_database_file()

    def get_keys(self):
        self._database = self._load_database_from_file()
        return [i for i in self._database.keys()]

    def is_set(self, key):
        self._database = self._load_database_from_file()
        return self._database[key] != ""

    def save_to_database_file(self):
        with open(self._database_file, "w") as f:
            json.dump(self._database, f)

    def reset_database(self):
        self._database = {}
        self.save_to_database_file()

    @property
    def database(self):
        self._database = self._load_database_from_file()
        return self._database

    @database.setter
    def database(self, database):
        if type(database) is not dict:
            raise TypeError("Database must be a dict.")
        self._database = database
        self.save_to_database_file()

    @property
    def database_file(self):
        return self._database_file
