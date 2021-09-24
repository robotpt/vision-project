import json
import os


class Interactions:

    ASK_TO_DO_SCHEDULED = "ask to do scheduled"
    EVALUATION = "evaluation"
    FIRST_INTERACTION = "first interaction"
    PERSEVERANCE = "perseverance"
    PROMPTED_INTERACTION = "prompted interaction"
    SCHEDULED_INTERACTION = "scheduled interaction"
    TOO_MANY_PROMPTED = "too many prompted"

    POSSIBLE_INTERACTIONS = [
        ASK_TO_DO_SCHEDULED,
        EVALUATION,
        FIRST_INTERACTION,
        PERSEVERANCE,
        PROMPTED_INTERACTION,
        SCHEDULED_INTERACTION,
        TOO_MANY_PROMPTED
    ]


class DatabaseKeys:
    ACT_RATINGS = "act ratings"
    ASK_IF_GIVEN_UP = "ask if given up"
    BEST_SCORE = "best score"
    CURRENT_EVAL_SCORE = "current eval score"
    CURRENT_READING_COLOR = "current reading color"
    CURRENT_READING_ID = "current reading id"
    CURRENT_READING_INDEX = "current reading index"
    DAYS_WITHOUT_MAGNIFIER = "days without magnifier"
    DID_USE_MAGNIFIER = "did use magnifier"
    # DIFFICULTY_LEVEL = "difficulty level"
    FEEDBACK_VIDEOS = "feedback videos"
    FIRST_INTERACTION_DATETIME = "first interaction datetime"
    GOAL_RATING = "goal rating"
    GOOD_TO_CHAT = "good to chat"
    GRIT_FEEDBACK_INDEX = "grit feedback index"
    IREST_READING_INDEX = "IReST reading index"
    IS_CONTINUE_PERSEVERANCE = "is continue perseverance"
    IS_DO_EVALUATION = "is do evaluation"
    IS_DO_EVAL_DURING_PROMPTED = "is do eval during prompted"
    IS_DO_MINDFULNESS = "is do mindfulness"
    IS_DONE_EVAL_TODAY = "is done eval today"
    IS_DONE_GOAL_SETTING_TODAY = "is done goal setting today"
    IS_DONE_MINDFULNESS_TODAY = "is done mindfulness today"
    IS_DONE_PERSEVERANCE_TODAY = "is done perseverance today"
    IS_DONE_PROMPTED_TODAY = "is done prompted today"
    IS_INTERACTION_FINISHED = "is interaction finished"
    IS_GIVEN_UP = "is given up"
    # IS_NEW_DIFFICULTY_LEVEL = "is new difficulty level"
    IS_OFF_CHECKIN = "is off checkin"
    IS_PROMPTED_BY_USER = "is prompted by user"
    IS_PUBLISHED_CHOICES_TODAY = "is published choices today"
    IS_START_PERSEVERANCE = "is start perseverance"
    IS_USED_MAGNIFIER_TODAY = "is used magnifier today"
    LAST_5_EVAL_SCORES = "last 5 eval scores"
    LAST_INTERACTION_DATETIME = "last interaction datetime"
    LAST_UPDATE_DATETIME = "last update datetime"
    LAST_SCORE = "last score"
    FEELINGS_INDEX = "feelings index"
    MINDFULNESS_INDEX = "mindfulness index"
    MINDFULNESS_RATING = "mindfulness rating"
    NEXT_CHECKIN_DATETIME = "next checkin datetime"
    NUM_OF_DAYS_SINCE_LAST_EVAL = "num of days since last eval"
    NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING = "num of days since last goal setting"
    NUM_OF_DAYS_SINCE_LAST_MINDFULNESS = "num of days since last mindfulness"
    NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE = "num of days since last perseverance"
    NUM_OF_DAYS_SINCE_LAST_PROMPT = "num of days since last prompt"
    NUM_OF_PROMPTED_TODAY = "num of prompted today"
    PERSEVERANCE_COUNTER = "perseverance counter"
    READING_TASK_DATA = "reading task data"
    READING_EVAL_INDEX = "reading eval index"
    READING_EVAL_TYPE = "reading eval type"
    SELF_REPORTS = "self reports"
    SPOT_READING_ANSWER = "spot reading answer"
    SRT_READING_INDEX = "SRT reading index"
    POST_SRT_INDEX = "post SRT index"
    SUNDAY_SCHEDULED_TASK = "sunday scheduled task"
    UNABLE_TO_READ = "unable to read"
    USER_NAME = "user name"
    VIDEO_INTRO_INDEX = "video intro index"
    VIDEO_TO_PLAY = "video to play"


# set up resources paths
cwd = os.path.dirname(os.path.abspath(__file__))
resources_directory = os.path.join(cwd, "..", "..", "..", "resources")
reading_task_data_path = os.path.join(resources_directory, "deployment", "reading_task_data.json")
with open(reading_task_data_path) as f:
    READING_TASK_DATA = json.load(f)


INITIAL_STATE_DB = {
    DatabaseKeys.ACT_RATINGS: {},
    DatabaseKeys.ASK_IF_GIVEN_UP: False,
    DatabaseKeys.BEST_SCORE: None,
    DatabaseKeys.CURRENT_EVAL_SCORE: 0,
    DatabaseKeys.CURRENT_READING_COLOR: None,
    DatabaseKeys.CURRENT_READING_ID: None,
    DatabaseKeys.CURRENT_READING_INDEX: 0,
    DatabaseKeys.DAYS_WITHOUT_MAGNIFIER: 0,
    DatabaseKeys.DID_USE_MAGNIFIER: True,
    # DatabaseKeys.DIFFICULTY_LEVEL: "1",
    DatabaseKeys.FEEDBACK_VIDEOS: {
        "distance 4x": "https://www.youtube.com/embed/tZyaCbmJOtY",
        "distance 6x": "https://www.youtube.com/embed/IzM6pxzoAKg",
        "light": "https://www.youtube.com/embed/1xK1m6U2HkE",
        "parallel": "https://www.youtube.com/embed/1lvG7ovNOH8",
        "steady": "https://www.youtube.com/embed/SDzg0_73izo",
        "upside down": "https://www.youtube.com/embed/udSDgQo9Ms8",
        "no video": ""
    },
    DatabaseKeys.FIRST_INTERACTION_DATETIME: None,
    DatabaseKeys.GOAL_RATING: 0,
    DatabaseKeys.GOOD_TO_CHAT: None,
    DatabaseKeys.GRIT_FEEDBACK_INDEX: 0,
    DatabaseKeys.IREST_READING_INDEX: -1,
    DatabaseKeys.IS_CONTINUE_PERSEVERANCE: None,
    DatabaseKeys.IS_DO_EVALUATION: None,
    DatabaseKeys.IS_DO_EVAL_DURING_PROMPTED: False,
    DatabaseKeys.IS_DO_MINDFULNESS: False,
    DatabaseKeys.IS_DONE_EVAL_TODAY: False,
    DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
    DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
    DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
    DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
    DatabaseKeys.IS_INTERACTION_FINISHED: False,
    DatabaseKeys.IS_GIVEN_UP: [],
    # DatabaseKeys.IS_NEW_DIFFICULTY_LEVEL: False,
    DatabaseKeys.IS_OFF_CHECKIN: None,
    DatabaseKeys.IS_PROMPTED_BY_USER: False,
    DatabaseKeys.IS_PUBLISHED_CHOICES_TODAY: False,
    DatabaseKeys.IS_START_PERSEVERANCE: False,
    DatabaseKeys.IS_USED_MAGNIFIER_TODAY: False,
    DatabaseKeys.LAST_5_EVAL_SCORES: [],
    DatabaseKeys.LAST_INTERACTION_DATETIME: None,
    DatabaseKeys.LAST_UPDATE_DATETIME: None,
    DatabaseKeys.LAST_SCORE: None,
    DatabaseKeys.FEELINGS_INDEX: None,
    DatabaseKeys.MINDFULNESS_INDEX: 0,
    DatabaseKeys.MINDFULNESS_RATING: 0,
    DatabaseKeys.NEXT_CHECKIN_DATETIME: None,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
    DatabaseKeys.NUM_OF_PROMPTED_TODAY: 0,
    DatabaseKeys.PERSEVERANCE_COUNTER: 0,
    DatabaseKeys.POST_SRT_INDEX: -1,
    DatabaseKeys.READING_TASK_DATA: READING_TASK_DATA,
    DatabaseKeys.READING_EVAL_INDEX: 1,
    DatabaseKeys.READING_EVAL_TYPE: None,
    DatabaseKeys.SELF_REPORTS: [],
    DatabaseKeys.SPOT_READING_ANSWER: None,
    DatabaseKeys.SRT_READING_INDEX: 0,
    DatabaseKeys.SUNDAY_SCHEDULED_TASK: None,
    DatabaseKeys.USER_NAME: "",
    DatabaseKeys.UNABLE_TO_READ: False,
    DatabaseKeys.VIDEO_INTRO_INDEX: 0,
    DatabaseKeys.VIDEO_TO_PLAY: None
}
