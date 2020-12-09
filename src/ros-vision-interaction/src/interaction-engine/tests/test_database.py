import atexit
import json
import mock
import os
import unittest

from interaction_engine.json_database import Database


class TestDatabase(unittest.TestCase):

    def setUp(self):
        self._database_file_name = "test_database.json"
        test_database = {
            "key1": "value1",
            "key2": "value2"
        }
        with open(self._database_file_name, "w") as f:
            json.dump(test_database, f)

    def tearDown(self) -> None:
        os.remove(self._database_file_name)

    def test_create_new_database_file(self):
        database = Database(self._database_file_name)

        self.assertEqual(
            self._database_file_name,
            database.database_file,
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
            )
        )

    def test_load_from_database_file(self):
        expected_database = {
            "key1": "value1",
            "key2": "value2"
        }
        with open(self._database_file_name, "w") as f:
            json.dump(expected_database, f)
        database = Database(self._database_file_name)
        self.assertEqual(expected_database, database.database)

    def test_load_from_empty_database_file(self):
        expected_database = {}
        with open(self._database_file_name, "w") as f:
            json.dump(expected_database, f)
        database = Database("test_database.json")
        self.assertEqual({}, database.database)

    def test_set_database(self):
        database = Database(self._database_file_name)
        database["key3"] = "value3"
        self.assertEqual("value3", database["key3"])

        database["key4"] = "value4"
        self.assertEqual("value4", database["key4"])

    def test_clear_database(self):
        expected_initial_database = {"key1": "value1", "key2": "value2"}
        expected_cleared_database = {"key1": "", "key2": ""}

        database = Database(self._database_file_name)
        self.assertEqual(expected_initial_database, database.database)

        database.clear_value("key1")
        database.clear_value("key2")
        self.assertEqual(expected_cleared_database, database.database)

        database.database = expected_initial_database
        self.assertEqual(expected_initial_database, database.database)

        database.clear_entire_database()
        self.assertEqual(expected_cleared_database, database.database)

    def test_access_same_file_from_multiple_databases(self):
        test_database = {"key1": "value1", "key2": "value2"}

        database1 = Database(self._database_file_name)
        database2 = Database(self._database_file_name)

        self.assertEqual(test_database, database1.database)
        self.assertEqual(test_database, database2.database)

        database1["key3"] = "value3"

        expected_database = {
            "key1": "value1",
            "key2": "value2",
            "key3": "value3",
        }
        self.assertEqual(expected_database, database1.database)
        self.assertEqual(expected_database, database2.database)


if __name__ == '__main__':
    unittest.main()
