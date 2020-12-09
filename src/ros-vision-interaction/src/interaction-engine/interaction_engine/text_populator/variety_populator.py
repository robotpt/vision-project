import json
import random

from robotpt_common_utils import lists, math_tools, strings
from interaction_engine.text_populator.base_populator import BasePopulator


class VarietyPopulator(BasePopulator):

    class Tags:
        MAIN = 'var'
        INDEX = 'index'
        IS_WRAP_INDEX = 'is_wrap_index'

    def __init__(
            self,
            files,
            wild_card_symbol='*',
    ):
        super().__init__(
            main_tags=self.Tags.MAIN,
            option_tags=[
                self.Tags.INDEX,
                self.Tags.IS_WRAP_INDEX
            ]
        )

        self._variations = VarietyPopulator._create_dict(files)
        self._wild_card_symbol = wild_card_symbol

    def get_replacement(
            self,
            key,
            index=None,
            is_wrap_index=True
    ):
        choices = self.values(key)
        if index is None:
            return random.choice(choices)
        else:
            if not is_wrap_index and index > self.get_num_variations(key):
                raise IndexError
            if math_tools.is_int(index):
                index = int(index)
            else:
                raise ValueError("Index must be an int")
            i = index % self.get_num_variations(key)
            return choices[i]

    def get_num_variations(self, key):
        return len(self.values(key))

    def __contains__(self, key):
        return key in self._variations

    @property
    def keys(self):
        return list(self._variations.keys())

    def values(self, key):
        active_keys = strings.wildcard_search_in_list(key, self.keys)
        values = []
        for active_key in active_keys:
            values += self._variations[active_key]
        return values

    @staticmethod
    def _create_dict(
        files,
        variations_dict=None,
    ):
        files = lists.make_sure_is_iterable(files)
        for f in files:
            with open(f) as variation_file:
                new_dict = json.load(variation_file)
                variations_dict = VarietyPopulator._combine_dicts(variations_dict, new_dict)
        return variations_dict

    @staticmethod
    def _combine_dicts(dict1, dict2):
        if dict1 is None:
            dict1 = {}
        if dict2 is None:
            dict2 = {}
        for key2 in dict2:
            if key2 in dict1:
                list1 = lists.make_sure_is_iterable(dict1[key2])
                list2 = lists.make_sure_is_iterable(dict2[key2])
                dict1[key2] = VarietyPopulator._combine_lists_without_duplicates(list1, list2)
            else:
                list2 = lists.make_sure_is_iterable(dict2[key2])
                dict1[key2] = VarietyPopulator._combine_lists_without_duplicates([], list2)
        return dict1

    @staticmethod
    def _combine_lists_without_duplicates(list1, list2):
        list1 = VarietyPopulator._remove_duplicates_from_list(list1)
        list2 = VarietyPopulator._remove_duplicates_from_list(list2)
        set_from_list = set().union(list1, list2)
        return list(set_from_list)

    @staticmethod
    def _remove_duplicates_from_list(li):
        return list(set(li))


if __name__ == '__main__':

    variation_file = 'variation.json'
    variation_dict = {
        "greeting": ["Hi", "Hello", "Hola"],
        "question": [
            "Do you like green?",
            "Do you like dogs?",
            "Do you like apples?",
            "Do you like me?"
        ],
        "foo": ["foo", "fake"],
        "foobar": "foo-bar",
        "fakebar": "fake-bar"
    }

    with open(variation_file, 'w', newline='') as f:
        json.dump(variation_dict, f)

    import os
    import atexit

    # atexit.register(lambda: os.remove(db_file))
    atexit.register(lambda: os.remove(variation_file))

    variety_populator = VarietyPopulator(variation_file)
    variations_dict_ = VarietyPopulator._create_dict(variation_file)
    print(variations_dict_)
