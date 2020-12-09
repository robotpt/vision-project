import json
import unittest
import os
from interaction_engine.json_database import Database
from interaction_engine.text_populator import TextPopulator
from interaction_engine.text_populator.database_populator import DatabasePopulator
from interaction_engine.text_populator.variety_populator import VarietyPopulator

db_file = 'test_db.json'

variation_file = 'variation.json'
variation_file_contents = {
    "greeting": ["Hi", "Hello", "Hola"],
    "question": [
        "I am the life of the party",
        "I am always prepared",
        "I get stressed out easily",
        "I have a rich vocabulary"
    ],
    "only one": "just one",
    "replace1": "{'var': 'replace2'}",
    "replace2": "{'var': 'replace3'}",
    "replace3": "Works!",
    "foo": ["foo", "fake"],
    "foobar": "foo-bar",
    "fakebar": "fake-bar"
}


class TestTextPopulator(unittest.TestCase):

    def setUp(self) -> None:

        db = Database(db_file)
        db["user_name"] = None
        db["question_idx"] = 0
        db["answers"] = None

        with open(variation_file, 'w', newline='') as f:
            json.dump(variation_file_contents, f)

        variety_populator_ = VarietyPopulator(variation_file)
        database_populator_ = DatabasePopulator(db_file)
        self._text_populator = TextPopulator(variety_populator_, database_populator_)
        self._db = db

    def tearDown(self) -> None:

        os.remove(db_file)
        os.remove(variation_file)

    def test_plain_text_entry(self):
        for t in ['text', 'a', '0', 'a sentence']:
            self.assertTrue(
                self._text_populator.is_valid(t)
            )
            self.assertEqual(
                t,
                self._text_populator.run(
                    t
                )
            )

    def test_replacement_in_string(self):
        for text_to_include in ['aeunheu neuhaen ', 'abc', 'a', '']:
            text = "{'var': 'only one'}"
            self.assertTrue(
                self._text_populator.is_valid(
                    text_to_include + text + text_to_include
                )
            )
            self.assertEqual(
                text_to_include + 'just one' + text_to_include,
                self._text_populator.run(
                    text_to_include + text + text_to_include
                )
            )

            text = "{'var': 'greeting'}"
            self.assertTrue(
                self._text_populator.is_valid(
                    text_to_include + text + text_to_include
                )
            )
            self.assertTrue(
                self._text_populator.run(
                    text_to_include + text + text_to_include
                ) in [
                    text_to_include + t + text_to_include for t in [
                        'Hi', 'Hello', 'Hola'
                    ]
                ]
            )

            text = "{greeting}"
            self.assertTrue(
                self._text_populator.is_valid(
                    text_to_include + text + text_to_include
                )
            )
            self.assertTrue(
                self._text_populator.run(
                    text_to_include + text + text_to_include
                ) in [
                    text_to_include + t + text_to_include for t in [
                        'Hi', 'Hello', 'Hola'
                    ]
                ]
            )

    def test_bad_string(self):
        for text_to_include in ['aeunheu neuhaen ', 'abc', 'a', '']:
            text = "{'not a key': 'greeting'}"
            self.assertRaises(
                KeyError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include,
            )
            self.assertRaises(
                KeyError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )

            text = "{'var: 'greeting'}"
            self.assertRaises(
                SyntaxError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include
            )
            self.assertRaises(
                SyntaxError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )
            text = "{var': 'greeting'}"
            self.assertRaises(
                SyntaxError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include
            )
            self.assertRaises(
                SyntaxError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )
            text = "'var': 'greeting'}"
            self.assertRaises(
                ValueError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include
            )
            self.assertRaises(
                ValueError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )
            text = "{'var': 'greeting'"
            self.assertRaises(
                ValueError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include
            )
            self.assertRaises(
                ValueError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )

    def test_bad_db_key(self):
        for text_to_include in ['aeunheu neuhaen ', 'abc', 'a', '']:
            text = "{'db': 'not a key'}"
            self.assertRaises(
                KeyError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include,
                )
            self.assertRaises(
                KeyError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )

    def test_bad_multiple_arg_key(self):
        for text_to_include in ['aeunheu neuhaen ', 'abc', 'a', '']:
            text = "{'db': 'user_name', 'foo': 'bar'}"
            self.assertRaises(
                KeyError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include,
                )
            self.assertRaises(
                KeyError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )

            text = "{'var': 'greeting', 'foo': 'bar'}"
            self.assertRaises(
                KeyError,
                self._text_populator.is_valid,
                text_to_include + text + text_to_include,
                )
            self.assertRaises(
                KeyError,
                self._text_populator.run,
                text_to_include + text + text_to_include
            )

    def test_nested_tags(self):
        test_str = (
                "{'var': 'question', 'index': " +
                "'{'db': 'question_idx', 'post-op': 'increment'}'}"
        )
        for i in range(4):
            for _ in range(10):
                self._text_populator.is_valid(test_str)
            self.assertEqual(
                i,
                self._db["question_idx"]
            )
            self._text_populator.run(test_str)
            self.assertEqual(
                i+1,
                self._db["question_idx"]
            )

        test_str = "{'var': '{'var': 'foo'}bar'}"
        for text_to_include in ['aeunheu neuhaen ', 'abc', 'a', '']:
            self.assertTrue(
                self._text_populator.run(
                    text_to_include + test_str + text_to_include
                ) in [
                    text_to_include + t + text_to_include for t in [
                        'foo-bar', 'fake-bar'
                    ]
                ]
            )

    def test_replace_tag_with_tag(self):

        test_str = "{'var': 'replace1'}"
        final_replacement = 'Works!'
        for text_to_include in ['aeunheu neuhaen ', 'abc', 'a', '']:
            self.assertEqual(
                text_to_include + final_replacement + text_to_include,
                self._text_populator.run(
                    text_to_include + test_str + text_to_include
                )
            )

    def test_invalid_variation_file_value(self):

        bad_variation_file = 'bad_variation.json'
        bad_variation_file_contents = {
            "greeting": [
                "Hi",
                "Hello{'var': 'foo'}"
            ]
        }
        db = Database(db_file)
        db["user_name"] = None
        db["question_idx"] = 0
        db["answers"] = None

        with open(bad_variation_file, 'w', newline='') as f:
            json.dump(bad_variation_file_contents, f)

        variety_populator_ = VarietyPopulator(bad_variation_file)
        database_populator_ = DatabasePopulator(db_file)
        self.assertRaises(
            KeyError,
            TextPopulator,
            variety_populator_,
            database_populator_,
        )
        os.remove(bad_variation_file)

    def test_partial_entry_in_variation_file(self):

        bad_variation_file = 'bad_variation.json'
        bad_variation_file_contents = {
            "greeting": [
                "Hi",
                "Hello{"
            ]
        }
        db = Database(db_file)
        db["user_name"] = None
        db["question_idx"] = 0
        db["answers"] = None

        with open(bad_variation_file, 'w', newline='') as f:
            json.dump(bad_variation_file_contents, f)

        variety_populator_ = VarietyPopulator(bad_variation_file)
        database_populator_ = DatabasePopulator(db_file)
        self.assertRaises(
            ValueError,
            TextPopulator,
            variety_populator_,
            database_populator_,
        )
        os.remove(bad_variation_file)

    def test_replacement_with_multi_arg_entry(self):
        correct_variation_file = 'correct_variation.json'
        correct_variation_file_contents = {
            "greeting": [
                "Hi ready for number {'db': 'question_idx', 'post-op': 'increment'}",
                "Hello {'db': 'user_name'}"
            ]
        }

        db = Database(db_file)
        db["user_name"] = None
        db["question_idx"] = 0
        db["answers"] = None

        with open(correct_variation_file, 'w', newline='') as f:
            json.dump(correct_variation_file_contents, f)

        variety_populator_ = VarietyPopulator(correct_variation_file)
        database_populator_ = DatabasePopulator(db_file)
        TextPopulator(
            variety_populator_,
            database_populator_,
        )
        os.remove(correct_variation_file)

    def test_bad_database_key(self):

        bad_variation_file = 'bad_variation.json'
        bad_variation_file_contents = {
            "greeting": ["Hi", "Hello {'db': 'not_a_key'}"]
        }
        db = Database(db_file)

        with open(bad_variation_file, 'w', newline='') as f:
            json.dump(bad_variation_file_contents, f)

        variety_populator_ = VarietyPopulator(bad_variation_file)
        database_populator_ = DatabasePopulator(db)
        self.assertRaises(
            KeyError,
            TextPopulator,
            variety_populator_,
            database_populator_,
        )
        os.remove(bad_variation_file)
