from interaction_engine.message import Message
from interaction_engine.state import State
from interaction_engine.state_collection import StateCollection


class Keys:
    IS_FIRST_INTERACTION = "is first interaction"
    ASK_NAME = "ask name"
    ASK_AGE = "ask age"
    CONFIRM_NAME_AND_AGE = "confirm"
    GREETING = "greeting"
    HOW_ARE_YOU = "how are you"
    GOOD_TO_HEAR = "good to hear"
    IM_SORRY = "i'm sorry"
    STAY_HYDRATED = "stay hydrated"
    GET_ENOUGH_SLEEP = "get enough sleep"
    CHECK_IN_AGAIN = "check in again"
    WEEKEND_GREETING = "weekend greeting"


database_keys = [
    Keys.IS_FIRST_INTERACTION,
    Keys.ASK_NAME,
    Keys.ASK_AGE,
    Keys.CONFIRM_NAME_AND_AGE,
    Keys.GREETING,
    Keys.HOW_ARE_YOU,
    Keys.GOOD_TO_HEAR,
    Keys.IM_SORRY,
    Keys.STAY_HYDRATED,
    Keys.GET_ENOUGH_SLEEP,
    Keys.CHECK_IN_AGAIN,
    Keys.WEEKEND_GREETING
]

exit_transition = {"Next": "exit"}

ask_name = State(
    name=Keys.ASK_NAME,
    message_type=Message.Type.TEXT_ENTRY,
    content="What's your name?",
    next_states=[Keys.ASK_AGE],
    transitions={" is my name.": Keys.ASK_AGE}
)
ask_age = State(
    name=Keys.ASK_AGE,
    message_type=Message.Type.TEXT_ENTRY,
    content="How old are you, {ask name}?",
    next_states=[Keys.CONFIRM_NAME_AND_AGE],
    database_keys_to_read=[Keys.ASK_NAME],
    transitions={" years old.": Keys.CONFIRM_NAME_AND_AGE}
)
confirm_name_and_age = State(
    name=Keys.CONFIRM_NAME_AND_AGE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="Your name is {ask name} and you are {ask age} years old, is that right?",
    next_states=[Keys.ASK_NAME, Keys.GREETING],
    transitions={"Yes": Keys.GREETING, "No": Keys.ASK_NAME},
    database_keys_to_read=[Keys.ASK_NAME, Keys.ASK_AGE]
)
greeting = State(
    name=Keys.GREETING,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["Hi {ask name}!", "Hello {ask name}!"],
    next_states=["exit"],
    transitions=exit_transition,
    database_keys_to_read=[Keys.ASK_NAME],
)

first_interaction_states = [
    ask_name,
    ask_age,
    confirm_name_and_age,
    greeting,
]

first_interaction = StateCollection(
    "first interaction",
    Keys.ASK_NAME,
    first_interaction_states
)

how_are_you = State(
    name=Keys.HOW_ARE_YOU,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["Hi {ask name}, how are you?", "Hello {ask name}, how are you doing today?"],
    next_states=[Keys.GOOD_TO_HEAR, Keys.IM_SORRY],
    transitions={"Good!": Keys.GOOD_TO_HEAR, "Not great.": Keys.IM_SORRY},
    database_keys_to_read=[Keys.ASK_NAME],
)

good_to_hear = State(
    name=Keys.GOOD_TO_HEAR,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["Good to hear, have a nice day!"],
    next_states=["exit"],
    transitions=exit_transition
)

im_sorry = State(
    name=Keys.IM_SORRY,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="I'm sorry to hear that, I hope you feel better!",
    next_states=["exit"],
    transitions=exit_transition
)

how_are_you_interaction_states = [
    how_are_you,
    good_to_hear,
    im_sorry,
]

how_are_you_interaction = StateCollection(
    "how are you interaction",
    Keys.HOW_ARE_YOU,
    how_are_you_interaction_states
)

stay_hydrated = State(
    name=Keys.STAY_HYDRATED,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="Are you staying hydrated, {ask name}?",
    next_states=[Keys.GET_ENOUGH_SLEEP],
    transitions={"Yes": Keys.GET_ENOUGH_SLEEP, "No": Keys.GET_ENOUGH_SLEEP},
    database_keys_to_read=[Keys.ASK_NAME],
)

get_enough_sleep = State(
    name=Keys.GET_ENOUGH_SLEEP,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="Are you getting at least 7 hours of sleep every night?",
    next_states=[Keys.CHECK_IN_AGAIN],
    transitions={"Yes": Keys.CHECK_IN_AGAIN, "No": Keys.CHECK_IN_AGAIN}
)

check_in_again = State(
    name=Keys.CHECK_IN_AGAIN,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="Let's talk again soon; how about the same time tomorrow?",
    next_states=["exit"],
    transitions={"Sure": "exit", "No, the day after": "exit"}
)

check_in_interaction_states = [
    stay_hydrated,
    get_enough_sleep,
    check_in_again
]

check_in_interaction = StateCollection(
    "check in",
    Keys.STAY_HYDRATED,
    check_in_interaction_states
)

weekend_greeting = State(
    name=Keys.WEEKEND_GREETING,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="How's your weekend going?",
    next_states=[Keys.GOOD_TO_HEAR, Keys.IM_SORRY],
    transitions={"Great!": Keys.GOOD_TO_HEAR, "Not too good": Keys.IM_SORRY},
)

weekend_interaction_states = [
    weekend_greeting,
    good_to_hear,
    im_sorry,
]

weekend_interaction = StateCollection(
    "weekend interaction",
    Keys.WEEKEND_GREETING,
    weekend_interaction_states
)