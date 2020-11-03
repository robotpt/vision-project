import json
import mock
import unittest

from interaction_engine.database import Database


class TestDatabase(unittest.TestCase):

    # def setUp(self):
    # self._database = Database("test_database.json")

    def test_check_default_database_keys(self):
        with mock.patch("__builtin__.open", mock.mock_open(read_data="data")):
            database = Database("test_database.json")
            self.assertEqual(database.database, {})

            valid_database_keys_list = [
                ["key1", "key2", "key3"],
                ["a", "b", "c"]
            ]
            for valid_database_keys in valid_database_keys_list:
                database = Database("test_database.json", valid_database_keys)
                # self.assertEqual(valid_database_keys, database.database.keys())

                for value in database.database.values():
                    self.assertEqual("", value)

            invalid_database_keys_list = [
                "key1",
                {"key1": "value1", "key2": "value2"}
            ]
            for invalid_database_keys in invalid_database_keys_list:
                self.assertRaises(
                    TypeError,
                    Database,
                    database_file="test_database.json",
                    default_database_keys=invalid_database_keys
                )

    def test_set_initial_db(self):
        test_data = {
            "key1": "value1",
            "key2": "value2",
            "key3": "value3",
        }
        read_data = json.dumps(test_data)
        with mock.patch("__builtin__.open", mock.mock_open(read_data=read_data)):
            database = Database("test_database.json")
            self.assertEqual(database.database, test_data)


if __name__ == '__main__':
    unittest.main()
