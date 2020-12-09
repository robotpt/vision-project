import ast
import random

from interaction_engine.text_populator.variety_populator import VarietyPopulator
from interaction_engine.text_populator.database_populator import DatabasePopulator
from robotpt_common_utils import lists


# TODO: Move kwargs handling inside of each populator class
# TODO: Allow populator to have n args put in that are each checked for keywords
# TODO: Test TextPopulator


class TextPopulator:

    def __init__(
            self,
            variety_populator,
            database_populator,
            open_symbol='{',
            closed_symbol='}',
    ):
        self._variety_populator = variety_populator

        self._database_populator = database_populator

        self._open_symbol = open_symbol
        self._closed_symbol = closed_symbol

        try:
            self._test_variety_populator(self._variety_populator)
            self._test_database_populator(self._database_populator)
        except Exception as e:
            raise e

    def run(self, text):
        return TextPopulator._parenthetic_processor(
            text,
            self._handle_string_input,
            open_symbol=self._open_symbol,
            closed_symbol=self._closed_symbol,
        )

    def is_valid(self, text):
        try:
            TextPopulator._parenthetic_processor(
                text,
                self._test_kwargs,
                open_symbol=self._open_symbol,
                closed_symbol=self._closed_symbol,
            )
        except Exception as e:
            raise e
        return True

    def _test_variety_populator(self, variety_populator):
        if type(variety_populator) is not VarietyPopulator:
            raise TypeError
        for key in variety_populator.keys:
            for value in variety_populator.values(key):
                self.is_valid(value)

    def _test_database_populator(self, database_populator):
        if type(database_populator) is not DatabasePopulator:
            raise TypeError

    def _test_kwargs(self, segment):
        # TODO: Test should be able to key unset db entries by checking if they exist, but are unset

        try:
            kwargs = ast.literal_eval(segment)
            if type(kwargs) is not dict:
                raise ValueError
        except ValueError:
            kwargs = {VarietyPopulator.Tags.MAIN: segment[1:-1]}

        if VarietyPopulator.Tags.MAIN in kwargs:

            if not self._variety_populator.is_tags_valid(kwargs):
                raise KeyError("Invalid key in database")

            key = kwargs[VarietyPopulator.Tags.MAIN]
            if key not in self._variety_populator:
                raise KeyError(f"'{key}' does not match any keys")

        elif DatabasePopulator.Tags.MAIN in kwargs:

            if not self._database_populator.is_tags_valid(kwargs):
                raise KeyError("Invalid key in database")

            key = kwargs[DatabasePopulator.Tags.MAIN]
            if key not in self._database_populator:
                raise KeyError(f"'{key}' has not been created yet")

        elif 'rand' in kwargs:
            if not lists.is_iterable(kwargs['rand']):
                raise ValueError("Rand must be iterable (not including string)")
        else:
            raise KeyError(f"No handler found for keys in '{kwargs.keys()}'")

        return "tested"

    def _handle_string_input(self, segment):

        try:
            kwargs = ast.literal_eval(segment)
        except ValueError:
            kwargs = {VarietyPopulator.Tags.MAIN: segment[1:-1]}

        if VarietyPopulator.Tags.MAIN in kwargs:

            if not self._variety_populator.is_tags_valid(kwargs):
                raise KeyError("Invalid key in database")

            handle = kwargs[VarietyPopulator.Tags.MAIN]

            if VarietyPopulator.Tags.INDEX in kwargs:

                if VarietyPopulator.Tags.IS_WRAP_INDEX in kwargs:
                    is_wrap_index = kwargs[VarietyPopulator.Tags.IS_WRAP_INDEX]
                else:
                    is_wrap_index = True

                return self._variety_populator.get_replacement(
                    handle,
                    index=kwargs[VarietyPopulator.Tags.INDEX],
                    is_wrap_index=is_wrap_index,
                )
            else:
                return self._variety_populator.get_replacement(handle)

        elif DatabasePopulator.Tags.MAIN in kwargs:

            if not self._database_populator.is_tags_valid(kwargs):
                raise KeyError("Invalid key in database")

            key = kwargs[DatabasePopulator.Tags.MAIN]

            if DatabasePopulator.Tags.DEFAULT_VALUE in kwargs:
                default_value = kwargs[DatabasePopulator.Tags.DEFAULT_VALUE]
            else:
                default_value = None

            if DatabasePopulator.Tags.POST_OP in kwargs:
                fn = kwargs[DatabasePopulator.Tags.POST_OP]
                value = self._database_populator.get_replacement(
                    key,
                    modify_before_resaving_fn=fn,
                    default_value=default_value,
                )
            else:
                value = self._database_populator.get_replacement(
                    key,
                    default_value=default_value,
                )

            return value

        elif 'rand' in kwargs:
            return random.choice(kwargs['rand'])
        else:
            raise KeyError(f"No handler found for keys in '{kwargs.keys()}'")

    @staticmethod
    def _parenthetic_processor(
            text,
            fn,
            open_symbol='{',
            closed_symbol='}',
    ):
        open_parenthesis_stack = []

        itr = 0
        while itr < len(text):

            if text[itr:itr+len(open_symbol)] == open_symbol:
                open_parenthesis_stack.append(itr)

            elif text[itr:itr+len(closed_symbol)] == closed_symbol:
                if len(open_parenthesis_stack) == 0:
                    raise ValueError("Not all closing symbols matched")

                start_idx = open_parenthesis_stack.pop()
                end_idx = itr+1
                segment = text[start_idx:end_idx]

                replacement = fn(segment)

                if replacement is not None:
                    text = "".join((
                        text[:start_idx],
                        replacement,
                        text[end_idx:]
                    ))
                    itr = start_idx-1

            itr += 1
        if len(open_parenthesis_stack) == 0:
            return text
        else:
            raise ValueError("Not all open symbols matched")


if __name__ == "__main__":

    import json
    import os
    from pickled_database import PickledDatabase

    db_file = 'test_db.json'
    db = PickledDatabase(db_file)
    db.create_key_if_not_exists('key1', 1)
    db.create_key_if_not_exists('key2', 'two')
    db.create_key_if_not_exists('no_value_key')
    db.create_key_if_not_exists('user_name', 'Audrow')
    db.create_key_if_not_exists('question_idx', 1)

    my_str = """
{'var': 'greeting'}, {'db': 'user_name'}. 
{'rand': ["What's up", 'How are you', "How's it going"]}?
{'var': 'question', 'index': '{'db': 'question_idx', 'post-op': 'increment'}'}
{'var': '{'var': 'foo'}bar'}
    """

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

    import atexit
    atexit.register(lambda: os.remove(db_file))
    atexit.register(lambda: os.remove(variation_file))

    variety_populator_ = VarietyPopulator(variation_file)
    database_populator_ = DatabasePopulator(db_file)

    text_populator = TextPopulator(variety_populator_, database_populator_)
    for _ in range(4):
        out = text_populator.run(my_str)
        print(out)
