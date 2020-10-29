#!/usr/bin/env python
import actionlib
import logging
import os
import random
import rospy

from interaction_engine.cordial_interface import CordialInterface
from interaction_engine.database import Database
from interaction_engine.int_engine import InteractionEngine
from interaction_engine.interface import Interface
from interaction_engine.message import Message
from interaction_engine.state import State
from interaction_engine.state_collection import StateCollection

from cordial_msgs.msg import AskOnGuiAction, AskOnGuiGoal, MouseEvent
from std_msgs.msg import Bool

logging.basicConfig(level=logging.INFO)

jokes = {
    "science": [
        "I'm reading a book about anti-gravity. <break time=\"0.75s\"/>It's impossible to put down!",
        "*QT/sad*Don't trust atoms. <break time=\"0.75s\"/>They make up everything!",
        "What's an astronaut's favorite part of a computer?<break time=\"0.75s\"/> *QT/happy*The space bar.",
        "Wanna hear a joke about paper?<break time=\"0.75s\"/>*QT/happy* Never mind - it's tearable.",
    ],
    "vision": [
        "Why did the cell phone go see an optometrist?<break time=\"0.75s\"/>*QT/happy* It needed contacts.",
        "*question_1*<prosody rate=\"slow\">What</prosody> do you call <break strength=\"weak\"/>an optometrist on a "
        "boat?*QT/happy* <break time=\"0.75s\"/>A see captain!",
        "What do you call a <prosody rate=\"60%\" pitch=\"medium\">deer</prosody><break time=\"0.2s\"/> with <prosody rate=\"slow\">no eyes</prosody>?<break time=\"0.75s\"/>*QT/happy* "
        "<prosody rate=\"x-slow\">No idea!</prosody>!",
        "What did the man with the magnifying glass say to the guy in the emergency room? ICU!",
        "Did you hear about the penny and magnifying glass who got married? Their wedding was magnify-cent.",
        "You know why women's eyes are so noticeable these days? It's the mask <break time=\"0.1s\"/>era during the pandemic.",
        "Did you hear that they make a webpage for people who suffer from chronic eye pain? It's a site for sore eyes.",
        "I love jokes about the eyes. The cornea the better."
    ],
    "robots": [
        "What is a robot's favorite snack? <break time=\"0.75s\"/>*QT/yawn*<prosody volume=\"loud\" rate=\"slow\">"
        "*happy*Computer</prosody> <prosody rate=\"slow\" pitch=\"high\">chips</prosody>. Chomp, chomp!",
        "What is it called when a robot eats a sandwich in one chomp?*QT/yawn*<break time=\"0.75s\"/> A megabyte.",
        "Why did the robot marry his fiancee? <break time=\"0.75s\"/>*QT/happy*He couldn't resistor!",
        "Why did the robot go to the shoe shop? To get rebooted.",
        "A robot's collection of musical instruments will never be complete. They can never get any organs."
    ]

}

jokes_prefaces = [
    "I <prosody rate=\"slow\">love</prosody> funny jokes about {category}, like this one.",
    "I happen to know a bunch of funny jokes about {category}; let me share one with you.",
    "Life is so much easier when you have a great sense of humor. Let me share a joke with you <prosody "
    "rate=\"slow\">about {category}</prosody>.",
]


def make_random_joke_content():
    joke_category = random.choice(jokes.keys())
    joke_content = random.choice(jokes_prefaces).format(category=joke_category)
    joke = random.choice(jokes[joke_category])
    joke_content += " <break time=\"1s\"/>" + joke
    jokes[joke_category].remove(joke)
    return joke_content


news_stories = [
    "<prosody rate=\"medium\">George Ahearn, who grew up in the farming town of Othello, Washington, co-founded EastWest Food Rescue after "
    "learning that Covid-19 was costing local farmers so much business that they were willing to destroy their crops. "
    "His non-profit has since moved three million pounds of produce from farms in eastern Washington to the western "
    "part of the state for distribution to hundreds of food banks and meal programs. Ahearn has a message for anyone "
    "who wants to make their community a better place, \"I have seen minutes of effort move thousands, and thousands "
    "of pounds of food. Just figure out what you are passionate about and what you could get involved in.\"</prosody>"
]

ASK_TO_CHAT = "ask to chat"
WHATS_YOUR_NAME = "what's your name"
HOW_ARE_YOU = "how are you"
IM_SORRY = "i'm sorry"
THATS_OK = "that's ok"
HOW_MANY_HOURS = "how many hours"
CANT_WAIT_TO_TALK = "can't wait to talk"
GLAD_TO_HEAR = "glad to hear"
DID_YOU_USE_MAGNIFIER = "did you use magnifier"
HOW_WAS_MAGNIFIER_EXPERIENCE = "how was magnifier experience"
EXCELLENT_RESPONSE = "response to excellent"
GOOD_RESPONSE = "response to good"
FAIR_RESPONSE = "response to fair"
POOR_RESPONSE = "response to poor"
JOKE_1 = "joke 1"
JOKE_2 = "joke 2"
JOKE_3 = "joke 3"
LIVEN_THINGS_UP = "liven things up"
SHOW_ME = "show me"
TAP_SCREEN = "tap screen"
HOW_DO_YOU_FEEL_ABOUT_VISION = "how do you feel about vision"
NO_CONCERNS_RESPONSE = "response to no concerns"
NO_NEWS_STORY_RESPONSE = "response to no news story"
TAKE_A_BREAK = "take a break"
ASK_TO_SHARE_NEWS_STORY = "ask to share news story"
NEWS_STORY = "share news story"
AFTER_NEWS_STORY = "after news story"
ENDING_JOKE = "ending joke"

ask_to_chat = State(
    name=ASK_TO_CHAT,
    content="<emphasis level=\"strong\"><prosody pitch=\"high\">Hello</prosody></emphasis>*QT/hi*<break "
            "strength=\"weak\"/> there! *question_1*Is this a <emphasis level=\"strong\"><prosody "
            "rate=\"slow\">good</prosody></emphasis> time to chat with me?",
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    next_states=[WHATS_YOUR_NAME, THATS_OK],
    transitions={
        "Yes": WHATS_YOUR_NAME,
        "No": THATS_OK
    }
)

whats_your_name = State(
    name=WHATS_YOUR_NAME,
    message_type=Message.Type.TEXT_ENTRY,
    content=["<emphasis level=\"strong\"> <prosody pitch=\"high\"> *happy* Hi!</prosody></emphasis> I'm <prosody "
             "pitch=\"medium\">Q</prosody><break strength=\"weak\"/>T<break strength=\"weak\"/> robot. "
             "*question_2*What's <emphasis level=\"strong\"><prosody rate=\"slow\">your</prosody></emphasis> name?"],
    next_states=[HOW_ARE_YOU],
    transitions={"Next": HOW_ARE_YOU},
)

how_are_you = State(
    name=HOW_ARE_YOU,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["It's nice to meet you {what's your name}. *question_1* How are you doing right now?"],
    next_states=[
        IM_SORRY,
        GLAD_TO_HEAR
    ],
    transitions={
        "Not so good": IM_SORRY,
        "Well": GLAD_TO_HEAR
    },
    database_keys_to_read=[WHATS_YOUR_NAME]
)

im_sorry = State(
    name=IM_SORRY,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["*QT/emotions/sad* I'm <prosody rate=\"slow\" pitch=\"low\">sorry</prosody> to hear that. Perhaps "
             "<prosody pitch=\"high\">I</prosody> can <prosody rate=\"slow\">help</prosody> a bit."],
    next_states=[JOKE_1],
    transitions={"Next": JOKE_1},
)

joke_1_content = random.choice(jokes["robots"])
jokes["robots"].remove(joke_1_content)

joke_1 = State(
    name=JOKE_1,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="The problem with me having a good sense of humor is that often the people I use it with are not in a "
            "very good mood. I wonder if I can make you chuckle a bit. " + joke_1_content,
    next_states=[DID_YOU_USE_MAGNIFIER],
    transitions={"Next": DID_YOU_USE_MAGNIFIER}
)

thats_ok = State(
    name=THATS_OK,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["That's OK {what's your name}, but let's plan to talk another time today or tomorrow. "
             "When is good for you?"],
    next_states=[CANT_WAIT_TO_TALK, HOW_MANY_HOURS],
    transitions={
        "Later today": HOW_MANY_HOURS,
        "Same time tomorrow": CANT_WAIT_TO_TALK
    },
    database_keys_to_read=[WHATS_YOUR_NAME]
)

cant_wait_to_talk = State(
    name=CANT_WAIT_TO_TALK,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["*happy**QT/happy*<emphasis level=\"strong\">Great</emphasis>, I can't <prosody pitch=\"high\">wait"
             "</prosody> to talk to you again <prosody pitch=\"high\" rate=\"slow\">then</prosody>."],
    next_states=[ENDING_JOKE],
    transitions={"Next": ENDING_JOKE},
)

glad_to_hear = State(
    name=GLAD_TO_HEAR,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["*happy* <emphasis level=\"strong\"> <prosody pitch=\"high\">Great</prosody></emphasis>, "
             "I'm <emphasis level=\"strong\">so glad</emphasis> to hear that things are ok with you."],
    next_states=[DID_YOU_USE_MAGNIFIER],
    transitions={"Next": DID_YOU_USE_MAGNIFIER},
)

did_you_use_magnifier = State(
    name=DID_YOU_USE_MAGNIFIER,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["<emphasis level=\"strong\"><prosody pitch=\"low\">Tell me</prosody></emphasis>*question_2*, "
             "did you use your magnifier today?"],
    next_states=[
        HOW_DO_YOU_FEEL_ABOUT_VISION,
        HOW_WAS_MAGNIFIER_EXPERIENCE
    ],
    transitions={
        "No": HOW_DO_YOU_FEEL_ABOUT_VISION,
        "Yes": HOW_WAS_MAGNIFIER_EXPERIENCE
    },
)

how_was_magnifier_experience = State(
    name=HOW_WAS_MAGNIFIER_EXPERIENCE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["{what's your name}, *question_2*how did it go with your magnifier?"],
    next_states=[
        EXCELLENT_RESPONSE,
        GOOD_RESPONSE,
        FAIR_RESPONSE,
        POOR_RESPONSE
    ],
    transitions={
        "Excellent": EXCELLENT_RESPONSE,
        "Good": GOOD_RESPONSE,
        "Fair": FAIR_RESPONSE,
        "Poor": POOR_RESPONSE
    },
    database_keys_to_read=[WHATS_YOUR_NAME]
)

excellent_response = State(
    name=EXCELLENT_RESPONSE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=[
        "<prosody pitch=\"high\" volume=\"loud\" rate=\"slow\">Fantastic*QT/happy*</prosody>*happy*! I'm so "
        "proud of you for doing <prosody pitch=\"high\" rate=\"slow\">so</prosody> well. Keep up the excellent work "
        "with your magnifier!"],
    next_states=[LIVEN_THINGS_UP],
    transitions={"Next": LIVEN_THINGS_UP},
)

good_response = State(
    name=GOOD_RESPONSE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=[
        "*happy*I am <prosody pitch=\"high\">glad</prosody> to hear it's going pretty well. Here's my advice: <break "
        "strength=\"x-strong\"/>let's aim to become <prosody pitch=\"high\">excellent</prosody> at using your "
        "magnifier. <prosody rate=\"slow\">I know</prosody> that you can get there! *happy*Your <prosody "
        "rate=\"slow\">future self</prosody> will thank you."],
    next_states=[LIVEN_THINGS_UP],
    transitions={"Next": LIVEN_THINGS_UP},
)

fair_response = State(
    name=FAIR_RESPONSE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=[
        "Let's see how we can make things better with practice. How about if we pick a number of tries and do that "
        "many each day. How about 2? It does not matter that each try be great, it's important that we give it a "
        "good try. How about that?"],
    next_states=[LIVEN_THINGS_UP],
    transitions={"Next": LIVEN_THINGS_UP},
)

poor_response = State(
    name=POOR_RESPONSE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=[
        "I am so sorry*sad* that it has been hard to use the magnifier! It takes time to get better at it. Let's set a "
        "goal for tomorrow to just give it a try and not feel bad about however it goes. Each try counts, "
        "so you should feel good about trying."],
    next_states=[LIVEN_THINGS_UP],
    transitions={"Next": LIVEN_THINGS_UP},
)

liven_things_up = State(
    name=LIVEN_THINGS_UP,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="Let's*QT/emotions/hoora* <prosody volume=\"loud\" rate=\"slow\" pitch=\"high\">liven</prosody> things up for a "
            "moment with a joke!",
    next_states=[JOKE_2],
    transitions={"Next": JOKE_2}
)

joke_2 = State(
    name=JOKE_2,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=make_random_joke_content(),
    next_states=[SHOW_ME],
    transitions={"Next": SHOW_ME}
)

show_me = State(
    name=SHOW_ME,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["Next, can you show me how you read something with your magnifier*question_1*?"],
    next_states=[
        TAP_SCREEN,
        TAKE_A_BREAK
    ],
    transitions={
        "Yes": TAP_SCREEN,
        "Not right now": TAKE_A_BREAK
    },
)

tap_screen = State(
    name=TAP_SCREEN,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["Tap my screen when you're done using your magnifier."],
    next_states=[TAKE_A_BREAK],
    transitions={"Next": TAKE_A_BREAK},
)

how_do_you_feel_about_vision = State(
    name=HOW_DO_YOU_FEEL_ABOUT_VISION,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["<prosody pitch=\"high\" rate=\"slow\">Oh</prosody>, I hope you'll find an opportunity to use it later "
             "today or tomorrow for sure. <break time=\"0.75s\"/>*question_1*<prosody rate=\"slow\">How</prosody> do "
             "you feel about your vision <prosody pitch=\"high\">today</prosody>?"],
    next_states=[
        TAKE_A_BREAK,
        NO_CONCERNS_RESPONSE
    ],
    transitions={
        "Frustrated": TAKE_A_BREAK,
        "No concerns": NO_CONCERNS_RESPONSE
    },
)

no_concerns_response = State(
    name=NO_CONCERNS_RESPONSE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["That's good to hear*happy*. Since you don't have any vision-related worries today:"],
    next_states=[ASK_TO_SHARE_NEWS_STORY],
    transitions={"Next": ASK_TO_SHARE_NEWS_STORY},
)

ask_to_share_news_story = State(
    name=ASK_TO_SHARE_NEWS_STORY,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["<emphasis level=\"strong\"><prosody pitch=\"high\">Next</prosody></emphasis>, I would like to "
             "share a <emphasis level=\"strong\">story</emphasis> with you. How does <prosody pitch=\"high\" "
             "rate=\"slow\">that sound</prosody>?"],
    next_states=[
        NO_NEWS_STORY_RESPONSE,
        NEWS_STORY
    ],
    transitions={
        "Not right now": NO_NEWS_STORY_RESPONSE,
        "Good": NEWS_STORY
    },
)

take_a_break = State(
    name=TAKE_A_BREAK,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["*QT/challenge*{what's your name}, I know it's <emphasis level=\"strong\"><prosody rate=\"medium\">more "
             "difficult</prosody></emphasis> to do things with your vision, but I know that together we <emphasis "
             "level=\"strong\"><prosody rate=\"slow\" pitch=\"high\">can</prosody></emphasis> get through it. "
             "<prosody pitch=\"high\"><prosody rate=\"slow\">For now</prosody></prosody>, let's take our minds "
             "<prosody rate=\"slow\">off</prosody> that."],
    next_states=[ASK_TO_SHARE_NEWS_STORY],
    transitions={"Next": ASK_TO_SHARE_NEWS_STORY},
    database_keys_to_read=[WHATS_YOUR_NAME]
)

joke_3 = State(
    name=JOKE_3,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=make_random_joke_content(),
    next_states=[ASK_TO_SHARE_NEWS_STORY],
    transitions={"Next": ASK_TO_SHARE_NEWS_STORY}
)

no_news_story_response = State(
    name=NO_NEWS_STORY_RESPONSE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=["<prosody pitch=\"low\" rate=\"slow\">No problem</prosody>, I understand. Let's plan to talk another "
             "time. <prosody rate=\"slow\">When</prosody> is good for "
             "<prosody pitch=\"high\" rate=\"slow\">you</prosody>?"],
    next_states=[CANT_WAIT_TO_TALK, HOW_MANY_HOURS],
    transitions={
        "Later today": HOW_MANY_HOURS,
        "Same time tomorrow": CANT_WAIT_TO_TALK
    },
)

how_many_hours = State(
    name=HOW_MANY_HOURS,
    message_type=Message.Type.TEXT_ENTRY,
    content="Please enter the number of hours from now<break time=\"0.5s\"/> as <prosody rate=\"slow\">(1-9)</prosody>.",
    next_states=[CANT_WAIT_TO_TALK],
    transitions={" hours": CANT_WAIT_TO_TALK}
)

news_story = State(
    name=NEWS_STORY,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=random.choice(news_stories),
    next_states=[AFTER_NEWS_STORY],
    transitions={"Next": AFTER_NEWS_STORY},
)

ending_joke_contents = [
    "Before we say goodbye, I'd like to tell you one of my favorite jokes about eyes and vision.<break time=\"1s\"/> "
    "*QT/emotions/shy* Knock knock, <break strength=\"x-strong\"/><prosody pitch=\"high\">who's there</prosody><break "
    "time=\"0.5s\"/>? Eyeball. <break time=\"0.5s\"/>Eyeball who? <break time=\"0.5s\"/> "
    "Eyeball my eyes out every time you go! <break strength=\"x-strong\"/>*QT/bye-bye**happy* Goodbye for now, {what's your name}!",
    "Before we say goodbye, I'd like to tell you one of my favorite jokes about robots.<break time=\"1s\"/> I'm so "
    "good at sleeping, I can do it with my eyes closed!*QT/bye-bye**happy* Goodbye for now, {what's your name}!"
]

ending_joke = State(
    name=ENDING_JOKE,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content=random.choice(ending_joke_contents),
    next_states=["exit"],
    transitions={"Bye!": "exit"},
    database_keys_to_read=[WHATS_YOUR_NAME]
)

after_news_story = State(
    name=AFTER_NEWS_STORY,
    message_type=Message.Type.MULTIPLE_CHOICE_ONE_COLUMN,
    content="I hope you enjoyed that <prosody pitch=\"high\">story!</prosody> Have a <prosody "
            "pitch=\"high\">great</prosody> rest of your day. And don't forget to use your *happy*<prosody "
            "pitch=\"high\">magnifier</prosody>! *QT/bye-bye*<break time=\"0.2s\"/><prosody pitch=\"high\" rate=\"slow\">Bye-bye!</prosody>",
    next_states=["exit"],
    transitions={"Bye!": "exit"},
)

init_state_name = ASK_TO_CHAT

states = [
    ask_to_chat,
    whats_your_name,
    how_are_you,
    im_sorry,
    thats_ok,
    cant_wait_to_talk,
    glad_to_hear,
    did_you_use_magnifier,
    how_was_magnifier_experience,
    excellent_response,
    good_response,
    fair_response,
    poor_response,
    joke_1,
    joke_2,
    joke_3,
    liven_things_up,
    show_me,
    tap_screen,
    how_do_you_feel_about_vision,
    no_concerns_response,
    no_news_story_response,
    take_a_break,
    ask_to_share_news_story,
    news_story,
    ending_joke,
    how_many_hours,
    after_news_story,
]

state_collection = StateCollection(
    name="state collection",
    init_state_name=init_state_name,
    states=states
)

default_database_keys = [state.name for state in states]

cwd = os.getcwd()
database_file = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "sar_demo.json"
)

database_manager = Database(
    database_file=database_file,
    default_database_keys=default_database_keys
)

interface = CordialInterface(
    action_name="cordial/say_and_ask_on_gui",
    seconds_until_timeout=None
)


is_record_publisher = rospy.Publisher("data_capture/is_record", Bool, queue_size=1)


if __name__ == "__main__":

    while not rospy.is_shutdown():

        # while not interface.is_begin_interaction:
        #     rospy.sleep(1)

        rospy.sleep(2)

        is_record_publisher.publish(True)

        interaction_engine = InteractionEngine(
            state_collection=state_collection,
            database_manager=database_manager,
            interface=interface
        )

        database_manager.clear_entire_database()
        interaction_engine.run()

        is_record_publisher.publish(False)

        rospy.sleep(20)

        # interface.is_begin_interaction = False
