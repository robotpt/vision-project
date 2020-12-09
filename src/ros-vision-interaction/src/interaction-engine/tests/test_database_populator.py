import unittest
import os

from interaction_engine.text_populator.database_populator import DatabasePopulator
from interaction_engine.json_database import Database


class TestDatabasePopulator(unittest.TestCase):

    def setUp(self):
        self.file = 'test_db.json'
        db = Database(self.file)
        db['key1'] = '1'
        db['key2'] = 'two'
        db['no_value_key'] = None
        db['key3'] = 0
        self.dp = DatabasePopulator(db)

    def tearDown(self):
        os.remove(self.file)

    def test_get_replacement(self):

        self.assertEqual(
            '1',
            self.dp.get_replacement('key1')
        )
        self.assertEqual(
            'two',
            self.dp.get_replacement('key2')
        )

    def test_key_with_no_value(self):
        self.assertRaises(
            KeyError,
            self.dp.get_replacement,
            'no_value_key'
        )
        self.assertEqual(
            '3',
            self.dp.get_replacement('no_value_key', default_value=3)
        )

    def test_key_that_doesnt_exist(self):
        self.assertRaises(
            KeyError,
            self.dp.get_replacement,
            'not_a_key'
        )

    def test_post_op(self):
        for i in range(10):
            self.assertEqual(
                i,
                self.dp._db['key3']
            )
            self.dp.get_replacement('key3', modify_before_resaving_fn='increment')

        for i in range(10):
            self.assertEqual(
                10-i,
                self.dp._db['key3']
            )
            self.dp.get_replacement('key3', modify_before_resaving_fn='decrement')
