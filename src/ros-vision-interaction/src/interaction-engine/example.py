import json
import os

from interaction_engine import InteractionEngine
from interaction_engine.json_database import Database
from interaction_engine.messager import Message, Node, DirectedGraph
from interaction_engine.planner import MessagerPlanner
from interaction_engine.text_populator import TextPopulator, DatabasePopulator, VarietyPopulator
from interaction_engine.interfaces import TerminalClientAndServerInterface

"""
CREATE RESOURCES USED TO POPULATE TEXT
"""

import atexit

# Create the database
db_file = 'test_db.json'
db = Database(db_file)
db["user_name"] = None
db["question_idx"] = 0
db["answers"] = None

# Create a file with text used for variation
variation_file = 'variation.json'
variation_file_contents = {
    "greeting": ["Hi", "Hello", "Hola"],
    "question": [
        "I am the life of the party",
        "I am always prepared",
        "I get stressed out easily",
        "I have a rich vocabulary"
    ],
    "foo": ["foo", "fake"],
    "foobar": "foo-bar",
    "fakebar": "fake-bar"
}

with open(variation_file, 'w', newline='') as f:
    json.dump(variation_file_contents, f)

# Delete the created files after this program runs
atexit.register(lambda: os.remove(db_file))
atexit.register(lambda: os.remove(variation_file))

"""
CREATE CONTENT
"""

variety_populator_ = VarietyPopulator(variation_file)
database_populator_ = DatabasePopulator(db_file)
text_populator = TextPopulator(variety_populator_, database_populator_)

greeting = DirectedGraph(
    name='greeting',
    nodes=[
        Node(
            name='greeting',
            content="{'var': 'greeting'}",
            options="{'var': 'greeting'}",
            message_type=Message.Type.MULTIPLE_CHOICE,
            result_convert_from_str_fn=str,
            text_populator=text_populator,
            transitions='exit'
        ),
    ],
    start_node='greeting'
)
basic_questions = DirectedGraph(
    name='intro',
    nodes=[
        Node(
            name='ask name',
            content="What's your name?",
            options='Okay',
            message_type=Message.Type.DIRECT_INPUT,
            result_db_key='user_name',
            result_convert_from_str_fn=str,
            tests=lambda x: len(x) > 1,
            error_message='Enter something with at least two letters',
            is_confirm=True,
            text_populator=text_populator,
            transitions='ask age'
        ),
        Node(
            name='ask age',
            content="Alright, {'db': 'user_name'}, how old are you?",
            options='years_old',
            message_type=Message.Type.DIRECT_INPUT,
            result_convert_from_str_fn=float,
            result_db_key='user_age',
            tests=[
                lambda x: x >= 0,
                lambda x: x <= 200,
            ],
            error_message='Enter a number between 0 and 200',
            text_populator=text_populator,
            transitions='how are they'
        ),
        Node(
            name='how are they',
            content='How are you?',
            options=['Good', 'Okay', 'Bad'],
            message_type=Message.Type.MULTIPLE_CHOICE,
            text_populator=text_populator,
            transitions=['exit'],
        ),
    ],
    start_node='ask name'
)
psych_question = DirectedGraph(
    name='questions',
    nodes=[
        Node(
            name='psych question',
            content=(
                    "How do you feel about the following statement? " +
                    "'{'var': 'question', 'index': " +
                    "'{'db': 'question_idx', 'post-op': 'increment'}'}'"
            ),
            options=[
                'Strongly agree',
                'Agree',
                'Neutral',
                'disagree',
                'Strongly disagree',
            ],
            message_type=Message.Type.MULTIPLE_CHOICE,
            result_db_key='answers',
            is_append_result=True,
            text_populator=text_populator,
            transitions='exit',
        ),
    ],
    start_node='psych question'
)
closing = DirectedGraph(
    name='closing',
    nodes=[
        Node(
            name='closing',
            content="Bye",
            options=['Bye', 'See ya!'],
            message_type=Message.Type.MULTIPLE_CHOICE,
            result_convert_from_str_fn=str,
            text_populator=text_populator,
            transitions='exit'
        ),
    ],
    start_node='closing'
)

"""
ORGANIZE CONTENT TO BE PLAYED
"""

interface_ = TerminalClientAndServerInterface(database=db)
graphs_ = [greeting, basic_questions, psych_question, closing]

# Create a plan
plan_ = MessagerPlanner(graphs_)
plan_.insert(greeting)
plan_.insert(basic_questions)
for _ in range(3):
    plan_.insert(psych_question)
plan_.insert(closing)

"""
RUN IT!
"""

engine = InteractionEngine(interface_, plan_, graphs_)
engine.run()

print("=========================")
print("Currently in the database")
print(db)
