from robotpt_common_utils import lists
from interaction_engine.text_populator.populator import TextPopulator
from interaction_engine.messager.base_messenger import BaseMessenger


class Message(BaseMessenger):

    class Type:
        MULTIPLE_CHOICE = "multiple choice"
        DIRECT_INPUT = "text entry"
        TIME_ENTRY = "time entry"
        SLIDER = "slider"

    def __init__(
            self,
            content,
            options,
            message_type,
            args=None,
            result_convert_from_str_fn=str,
            result_db_key=None,
            is_append_result=False,
            tests=None,
            is_confirm=False,
            error_message="Please enter a valid input",
            error_options=('Okay', 'Oops'),
            text_populator=None,
    ):
        if text_populator is not None and type(text_populator) is not TextPopulator:
            raise ValueError
        self._text_populator = text_populator

        try:
            self._test_markup(content)
        except Exception as e:
            raise e
        self._content = content
        super().__init__(self._content)

        self._options = None
        self.options = options

        self._args = []
        self._last_args = None
        if args is not None:
            self.args = args

        self._message_type = message_type

        self._result_convert_from_str_fn = result_convert_from_str_fn
        self._result_db_key = result_db_key
        self._is_append_result = is_append_result

        if tests is not None:
            tests = lists.make_sure_is_iterable(tests)
        self._tests = tests

        self._is_confirm = is_confirm

        self._error_message = error_message
        self._error_options = error_options

        # Used to make message type able to be played in engine
        self._is_active = None
        self._last_options = None
        self.reset()

    @property
    def is_active(self) -> bool:
        return self._is_active

    def reset(self):
        self._is_active = True
        self._last_options = None

    def get_message(self):
        return self

    def transition(self, _) -> None:
        self._is_active = False

    def _test_markup(self, text):

        if self._text_populator is None:
            return True

        if callable(text):
            text = text()

        text = lists.make_sure_is_iterable(text)
        return all([self._text_populator.is_valid(t) for t in text])

    def _markup(self, text):

        if self._text_populator is None:
            return text

        if callable(text):
            text = text()

        text = lists.make_sure_is_iterable(text)
        result = [self._text_populator.run(t) for t in text]
        if len(result) is 1:
            return result[0]
        else:
            return result

    @property
    def content(self):
        return self._markup(self._content)

    @property
    def options(self):
        self._last_options = lists.make_sure_is_iterable(
            self._markup(self._options)
        )
        return self._last_options

    @property
    def last_options(self):
        return self._last_options

    @options.setter
    def options(self, options):
        try:
            self._test_markup(options)
        except Exception as e:
            raise e
        self._options = options

    @property
    def args(self):
        args = lists.make_sure_is_iterable(self._args)
        self._last_args = [self._markup(arg) for arg in args]
        return self._last_args

    @property
    def last_args(self):
        return self._last_args

    @args.setter
    def args(self, args):
        args = lists.make_sure_is_iterable(args)
        try:
            for arg in args:
                self._test_markup(arg)
        except Exception as e:
            raise Exception from e
        self._args = args

    @property
    def message_type(self):
        return self._message_type

    @property
    def result_type(self):
        return self._result_convert_from_str_fn

    @property
    def result_db_key(self):
        return self._result_db_key

    @property
    def is_append_result(self):
        return self._is_append_result

    @property
    def tests(self):
        return self._tests

    @property
    def error_message(self):
        return Message(
            content=self._error_message,
            options=self._error_options,
            message_type=Message.Type.MULTIPLE_CHOICE,
            text_populator=self._text_populator
        )

    @property
    def is_confirm(self):
        return self._is_confirm


if __name__ == "__main__":

    import json
    import os
    from interaction_engine.json_database import Database
    from interaction_engine.text_populator import DatabasePopulator
    from interaction_engine.text_populator.variety_populator import VarietyPopulator

    db_file = 'test_db.json'
    db = Database(db_file)
    db['key1'] = 1
    db['key2'] = 'two'
    db['no_value_key'] = None
    db['user_name'] = 'Audrow'
    db['question_idx'] = 1

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

    no_variation_msg = Message(
        content='Here is a question, or is it?',
        options='Not sure',
        message_type=Message.Type.MULTIPLE_CHOICE,
        text_populator=text_populator
    )
    print(no_variation_msg.content)
    print(no_variation_msg.options)

    variation_msg = Message(
        content=my_str,
        options=["{'rand': ['Yes', 'Definitely', 'Ofcourse']}", "{'rand': ['No', 'No way']}"],
        message_type=Message.Type.MULTIPLE_CHOICE,
        text_populator=text_populator
    )
    print(variation_msg.content)
    print(variation_msg.options)
