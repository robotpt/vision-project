class Interactions:

    ASK_TO_DO_SCHEDULED = "ask to do scheduled"
    EVALUATION = "evaluation"
    FIRST_INTERACTION = "first interaction"
    PROMPTED_INTERACTION = "prompted interaction"
    SCHEDULED_INTERACTION = "scheduled interaction"
    TOO_MANY_PROMPTED = "too many prompted"

    POSSIBLE_INTERACTIONS = [
        ASK_TO_DO_SCHEDULED,
        EVALUATION,
        FIRST_INTERACTION,
        PROMPTED_INTERACTION,
        SCHEDULED_INTERACTION,
        TOO_MANY_PROMPTED
    ]


class DatabaseKeys:
    BEST_SCORE = "best score"
    CURRENT_EVAL_SCORE = "current eval score"
    CURRENT_READING_COLOR = "current reading color"
    CURRENT_READING_ID = "current reading id"
    CURRENT_READING_INDEX = "current reading index"
    DIFFICULTY_LEVEL = "difficulty level"
    FEEDBACK_VIDEOS = "feedback videos"
    FIRST_INTERACTION_DATETIME = "first interaction datetime"
    GOOD_TO_CHAT = "good to chat"
    GRIT_FEEDBACK_INDEX = "grit feedback index"
    IS_CONTINUE_PERSEVERANCE = "is continue perseverance"
    IS_DO_EVALUATION = "is do evaluation"
    IS_DONE_EVAL_TODAY = "is done eval today"
    IS_DONE_GOAL_SETTING_TODAY = "is done goal setting today"
    IS_DONE_MINDFULNESS_TODAY = "is done mindfulness today"
    IS_DONE_PERSEVERANCE_TODAY = "is done perseverance today"
    IS_DONE_PROMPTED_TODAY = "is done prompted today"
    IS_INTERACTION_FINISHED = "is interaction finished"
    IS_NEW_DIFFICULTY_LEVEL = "is new difficulty level"
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
    USER_NAME = "user name"
    VIDEO_TO_PLAY = "video to play"


READING_TASK_DATA = {
    "IReST": {
        # difficulty levels
        "1": {
            # reading task IDs
            "100": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "101": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "102": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "103": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "104": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "105": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
        }
    },
    "MNread": {
        # difficulty levels
        "1": {
            # reading task IDs
            "200": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "201": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
        }
    },
    "SKread": {
        "1": {
            # reading task IDs
            "300": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "301": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
        }
    },
    "spot reading": {
        # difficulty levels
        "1": {
            # reading task IDs
            "sr11": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "sr12": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            }
        },
        "2": {
            "sr21": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "sr22": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            }
        },
        "3": {
            "sr31": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            }
        },
        "4": {
            "sr41": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "sr42": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
            "sr43": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            }
        }
    },
    "SRT": {
        # difficulty levels
        "1": {
            # reading task IDs
            "srt11": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
        }
    },
    "self selected": {
        # difficulty levels
        "1": {
            # reading task IDs
            "ss11": {
                "word_count": 50,
                "score": None,
                "color": "blue"
            },
        }
    }
}


INITIAL_STATE_DB = {
    DatabaseKeys.BEST_SCORE: None,
    DatabaseKeys.CURRENT_EVAL_SCORE: 0,
    DatabaseKeys.CURRENT_READING_COLOR: None,
    DatabaseKeys.CURRENT_READING_ID: None,
    DatabaseKeys.CURRENT_READING_INDEX: None,
    DatabaseKeys.DIFFICULTY_LEVEL: "1",
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
    DatabaseKeys.GOOD_TO_CHAT: None,
    DatabaseKeys.GRIT_FEEDBACK_INDEX: 0,
    DatabaseKeys.IS_CONTINUE_PERSEVERANCE: None,
    DatabaseKeys.IS_DO_EVALUATION: None,
    DatabaseKeys.IS_DONE_EVAL_TODAY: False,
    DatabaseKeys.IS_DONE_GOAL_SETTING_TODAY: False,
    DatabaseKeys.IS_DONE_MINDFULNESS_TODAY: False,
    DatabaseKeys.IS_DONE_PERSEVERANCE_TODAY: False,
    DatabaseKeys.IS_DONE_PROMPTED_TODAY: False,
    DatabaseKeys.IS_INTERACTION_FINISHED: False,
    DatabaseKeys.IS_NEW_DIFFICULTY_LEVEL: False,
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
    DatabaseKeys.NEXT_CHECKIN_DATETIME: None,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE: 0,
    DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT: 0,
    DatabaseKeys.NUM_OF_PROMPTED_TODAY: 0,
    DatabaseKeys.PERSEVERANCE_COUNTER: 0,
    DatabaseKeys.READING_TASK_DATA: READING_TASK_DATA,
    DatabaseKeys.READING_EVAL_INDEX: 0,
    DatabaseKeys.READING_EVAL_TYPE: None,
    DatabaseKeys.USER_NAME: "",
    DatabaseKeys.VIDEO_TO_PLAY: None
}
