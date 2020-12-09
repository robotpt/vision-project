from interaction_engine.interfaces.client_and_server_interface import ClientAndServerInterface
from interaction_engine.messager.message import Message
from interaction_engine.json_database import Database
from robotpt_common_utils import math_tools, lists
import textwrap


class TerminalClientAndServerInterface(ClientAndServerInterface):

    def __init__(
            self,
            database=None,
            is_create_db_key_if_not_exist=True,
    ):
        fns = [
            print_multiple_choice,
            input_multiple_choice,
            print_content,
            direct_input,
        ]
        super().__init__(
            *fns,
            database=database,
            is_create_db_key_if_not_exist=is_create_db_key_if_not_exist
        )


def print_content(message, width=50):

    if type(message) is not Message:
        raise ValueError("Must input message class")

    print("=====================")
    wrapper = textwrap.TextWrapper(width=width)
    text_list = wrapper.wrap(text=message.content)

    for element in text_list:
        print(element)


def print_multiple_choice(message):

    print_content(message)
    _print_enumerated_list(message.options)


def _print_enumerated_list(options):
    options = lists.make_sure_is_iterable(options)
    for i in range(len(options)):
        print(f" {i}. " + options[i])


def input_multiple_choice(message):

    if type(message) is not Message:
        raise ValueError("Must input message class")

    response = None
    while response not in range(len(message.options)):
        response_str = input(">>> ")
        if math_tools.is_int(response_str):
            response = int(response_str)

    return message.options[response]


def direct_input(_):
    response = ''
    while len(response) is 0:
        response = input(">>> ")

    return response


if __name__ == '__main__':

    multiple_choice_message1 = Message(
        content='How are you?',
        options=['Good', 'Okay', 'Bad'],
        message_type=Message.Type.MULTIPLE_CHOICE,
    )
    multiple_choice_message2 = Message(
        content='Do you love me?',
        options='Yes!',
        message_type=Message.Type.MULTIPLE_CHOICE,
        is_confirm=True,
    )
    real_number_entry_message = Message(
        content='How old are you?',
        options='years_old',
        message_type=Message.Type.DIRECT_INPUT,
        result_convert_from_str_fn=float,
        result_db_key='user_age',
        tests=[
            lambda x: x >= 0,
            lambda x: x <= 120,
        ],
        error_message='Enter a number between 0 and 120',
    )
    string_entry_message = Message(
        content="What's your name?",
        options='Okay',
        message_type=Message.Type.DIRECT_INPUT,
        result_convert_from_str_fn=str,
        result_db_key='user_name',
        tests=lambda x: len(x) > 1,
        error_message='Enter something with at least two letters',
        is_confirm=True,
    )

    import os
    import atexit

    db_file = "memory.json"
    atexit.register(lambda: os.remove(db_file))

    db = Database(db_file)
    interface = TerminalClientAndServerInterface(database=db)
    for msg in [
        multiple_choice_message1,
        multiple_choice_message2,
        real_number_entry_message,
        string_entry_message,
    ]:
        out = interface.run(msg)
        print(out)

    print("=========================")
    print("Currently in the database")
    print(db)
