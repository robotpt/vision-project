import logging
import os

from interaction_engine.message import Message
from interaction_engine.state import State
from interaction_engine.state_collection import StateCollection
from interaction_engine.int_engine import InteractionEngine
from interaction_engine.database import Database
from interaction_engine.terminal_interface import TerminalInterface

logging.basicConfig(level=logging.INFO)

# setting up content
cwd = os.getcwd()
database_file = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    'terminal_example_database.json'
)

ask_name = State(
    name="ask name",
    message_type=Message.Type.TEXT_ENTRY,
    content="What's your name?",
    next_states=["ask age"],
    transitions={" is my name.": "ask age"}
)
ask_age = State(
    name="ask age",
    message_type=Message.Type.TEXT_ENTRY,
    content="How old are you, {ask name}?",
    next_states=["confirm"],
    database_keys_to_read=["ask name"],
    transitions={" years old.": "confirm"}
)
confirm_name_and_age = State(
    name="confirm",
    message_type=Message.Type.MULTIPLE_CHOICE,
    content="Your name is {ask name} and you are {ask age} years old, is that right?",
    next_states=["ask name", "greeting"],
    transitions={"Yes": "greeting", "No": "ask name"},
    database_keys_to_read=["ask name", "ask age"]
)
greeting = State(
    name="greeting",
    message_type=Message.Type.NO_INPUT,
    content=["Hi {ask name}!", "Hello {ask name}!"],
    next_states=["breakfast"],
    database_keys_to_read=["ask name"],
)
breakfast = State(
    name="breakfast",
    message_type=Message.Type.MULTIPLE_CHOICE,
    content="Did you eat breakfast today?",
    next_states=["good", "not good"],
    transitions={"Yes": "good", "No": "not good"}
)
good = State(
    name="good",
    message_type=Message.Type.NO_INPUT,
    content=["Good!", "Good to hear!"],
    next_states=["water"],
)
not_good = State(
    name="not good",
    message_type=Message.Type.NO_INPUT,
    content=["That's not good."],
    next_states=["water"]
)
water = State(
    name="water",
    message_type=Message.Type.MULTIPLE_CHOICE,
    content=["Are you staying hydrated?"],
    next_states=["take care"],
    transitions={"Yes": "take care", "No": "take care"}
)
take_care = State(
    name="take care",
    message_type=Message.Type.MULTIPLE_CHOICE,
    content="Don't forget to take care of yourself!",
    next_states=["goodbye"],
    transitions={"I will, thanks!": "goodbye"}
)
goodbye = State(
    name="goodbye",
    message_type=Message.Type.MULTIPLE_CHOICE,
    content=["Bye!", "Goodbye!", "Have a nice day!"],
    next_states=["exit"],
    transitions={"Bye!": "exit"}
)

states = [
    ask_name,
    ask_age,
    confirm_name_and_age,
    greeting,
    breakfast,
    good,
    not_good,
    water,
    take_care,
    goodbye,
]

init_state_name = "ask name"

state_collection1 = StateCollection(
    name="state_collection1",
    init_state_name=init_state_name,
    states=states
)

default_database_keys = [state.name for state in states]

database_manager = Database(
    database_file=database_file,
    default_database_keys=default_database_keys
)

interface = TerminalInterface()

interaction_engine = InteractionEngine(
    state_collection=state_collection1,
    database_manager=database_manager,
    interface=interface
)

print(interaction_engine.database_manager.database)
interaction_engine.run()
