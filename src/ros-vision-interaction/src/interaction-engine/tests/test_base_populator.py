import unittest

from interaction_engine.text_populator.base_populator import BasePopulator
from robotpt_common_utils import lists

main_tags = ['foo', 'bar']
option_tags = ['baz', 'bop', 'woah', 'der', 'another', 'tag']


class FooPopulator(BasePopulator):
    def __init__(self):
        super().__init__(main_tags, option_tags)

    def get_replacement(self, key):
        pass


def make_dict_with_foo_elements(keys):
    keys = lists.make_sure_is_iterable(keys)
    if len(keys) is not len(set(keys)):
        raise ValueError("All elements in keys must be unique")
    return dict(zip(keys, range(len(keys))))


class TestBasePopulator(unittest.TestCase):

    def setUp(self) -> None:

        self.foo_populator = FooPopulator()

    def test_single_main_keys(self):

        for m in main_tags:
            self.assertTrue(
                self.foo_populator.is_tags_valid(
                    make_dict_with_foo_elements(m)
                )
            )
        for k in ['not a key', None, int, input]:
            self.assertFalse(
                self.foo_populator.is_tags_valid(
                    make_dict_with_foo_elements(k)
                )
            )

    def test_multiple_main_keys(self):
        for i in range(2, len(main_tags)):
            self.assertFalse(
                self.foo_populator.is_tags_valid(
                    make_dict_with_foo_elements(
                        main_tags[:i]
                    )
                )
            )

    def test_single_main_and_options_tags(self):
        for m in main_tags:
            for o in option_tags:
                self.assertTrue(
                    self.foo_populator.is_tags_valid(
                        make_dict_with_foo_elements([m, o])
                    )
                )
            for o in ['not a key', None, int, input]:
                self.assertFalse(
                    self.foo_populator.is_tags_valid(
                        make_dict_with_foo_elements([m, o])
                    )
                )

    def test_multiple_options_tags(self):

        for m in main_tags:
            for i in range(2, len(option_tags)):
                self.assertFalse(
                    self.foo_populator.is_tags_valid(
                        make_dict_with_foo_elements(
                            option_tags[:i].append(m)
                        )
                    )
                )
