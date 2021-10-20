import datetime
import logging
import vision_project_tools.reading_task_tools as reading_task_tools

from interaction_engine.messager.directed_graph import DirectedGraph
from interaction_engine.messager.node import Node
from interaction_engine.text_populator import TextPopulator
from interaction_engine.text_populator import DatabasePopulator
from interaction_engine.text_populator import VarietyPopulator

from vision_project_tools.constants import DatabaseKeys
from vision_project_tools.reading_task_tools import TaskDataKeys

logging.basicConfig(level=logging.INFO)


class InteractionBuilder:
    class Graphs:
        ASK_FOR_EVAL_DURING_PROMPTED = "ask for eval during prompted"
        ASK_IF_GIVEN_UP = "ask if given up"
        ASK_TO_DO_EVALUATION = "ask to do evaluation"
        ASK_TO_DO_PERSEVERANCE = "ask to do perseverance"
        ASK_TO_DO_SCHEDULED = "ask to do scheduled"
        CLOSING = "closing"
        CONTINUE_PERSEVERANCE = "is continue perseverance"
        EVALUATION = "evaluation"
        FEEDBACK_VIDEO = "feedback video"
        FIRST_SCHEDULED_CHECKIN = "first scheduled checkin"
        GOAL_SETTING = "goal setting"
        GRIT_TRANSITION = "grit transition"
        INTRODUCE_EVALUATION = "introduce evaluation"
        INTRODUCE_MINDFULNESS = "introduce mindfulness"
        INTRODUCE_QT = "introduce QT"
        LAST_EVALUATION = "last evaluation"
        MINDFULNESS_BODY_SCAN = "mindfulness body"
        MINDFULNESS_BREATHING = "mindfulness breathing"
        MINDFULNESS_DRINKING = "mindfulness drinking"
        NO_FEEDBACK_VIDEO = "no feedback video"
        NO_MAGNIFIER_USE = "no magnifier use"
        PERSEVERANCE = "perseverance"
        PLAN_CHECKIN_TOMORROW = "plan tomorrow's checkin"
        PLAN_NEXT_CHECKIN = "plan next checkin"
        POST_EVALUATION = "post evaluation"
        POST_IREST = "post IReST"
        POST_SSRT = "post SSRT"
        PROMPTED_ASK_TO_CHAT = "prompted ask to chat"
        PROMPTED_CHECKIN = "prompted checkin"
        PROMPTED_PLAN_NEXT_CHECKIN = "prompted plan next checkin"
        REMINDER_FOR_PROMPTED = "reminder for prompted"
        RETRY_SPOT_READING = "retry spot reading"
        REVISIT_MINDFULNESS = "revisit mindfulness"
        REWARD = "reward"
        SCHEDULED_ASK_TO_CHAT = "scheduled ask to chat"
        SCHEDULED_CHECKIN = "scheduled checkin"
        SPOT_READING_EVAL = "spot reading evaluation"
        SPOT_READING_FEEDBACK = "spot reading feedback"
        STORIES_AND_JOKES = "stories and jokes"
        TOO_MANY_PROMPTED = "too many prompted"

        POSSIBLE_GRAPHS = [
            ASK_FOR_EVAL_DURING_PROMPTED,
            ASK_IF_GIVEN_UP,
            ASK_TO_DO_EVALUATION,
            ASK_TO_DO_PERSEVERANCE,
            ASK_TO_DO_SCHEDULED,
            CLOSING,
            CONTINUE_PERSEVERANCE,
            EVALUATION,
            FEEDBACK_VIDEO,
            FIRST_SCHEDULED_CHECKIN,
            GOAL_SETTING,
            GRIT_TRANSITION,
            INTRODUCE_EVALUATION,
            INTRODUCE_MINDFULNESS,
            INTRODUCE_QT,
            LAST_EVALUATION,
            MINDFULNESS_BODY_SCAN,
            MINDFULNESS_BREATHING,
            MINDFULNESS_DRINKING,
            NO_FEEDBACK_VIDEO,
            NO_MAGNIFIER_USE,
            PERSEVERANCE,
            PLAN_NEXT_CHECKIN,
            PLAN_CHECKIN_TOMORROW,
            POST_EVALUATION,
            POST_IREST,
            POST_SSRT,
            PROMPTED_ASK_TO_CHAT,
            PROMPTED_CHECKIN,
            PROMPTED_PLAN_NEXT_CHECKIN,
            REMINDER_FOR_PROMPTED,
            RETRY_SPOT_READING,
            REVISIT_MINDFULNESS,
            REWARD,
            SCHEDULED_ASK_TO_CHAT,
            SCHEDULED_CHECKIN,
            SPOT_READING_EVAL,
            SPOT_READING_FEEDBACK,
            STORIES_AND_JOKES,
            TOO_MANY_PROMPTED
        ]

    def __init__(
            self,
            interaction_dict,
            variations_files,
            statedb,
            speaking_rate="slow"
    ):
        self._interaction_dict = interaction_dict
        self._variations_files = variations_files

        self._statedb = statedb

        self._database_populator = DatabasePopulator(self._statedb)
        self._variety_populator = VarietyPopulator(self._variations_files)

        self._text_populator = TextPopulator(self._variety_populator, self._database_populator)
        self._interactions = {}
        for graph_name in InteractionBuilder.Graphs.POSSIBLE_GRAPHS:
            self._interactions[graph_name] = self.build_graph_from_dict(
                self._interaction_dict,
                graph_name,
                self._text_populator,
                speaking_rate=speaking_rate
            )

        self._possible_graphs = [graph for graph in self._interactions.values()]

    def build_graph_from_dict(
            self,
            interactions_dict,
            graph_name,
            text_populator=None,
            speaking_rate="slow"
    ):
        graph_dict = interactions_dict[graph_name]
        start_node_name = graph_dict["start_node_name"]
        nodes = []

        valid_speaking_rates = [
            "x-slow",
            "slow",
            "medium",
            "fast",
            "x-fast"
        ]
        if speaking_rate is not None and speaking_rate not in valid_speaking_rates:
            raise ValueError("Not a valid speaking rate")

        all_nodes_in_graph = graph_dict["nodes"]
        for node_name in all_nodes_in_graph.keys():
            node_info = all_nodes_in_graph[node_name]
            optional_values = {
                "args": None,
                "result_convert_from_str_fn": str,
                "result_db_key": None,
                "is_append_result": False,
                "tests": None,
                "is_confirm": False,
                "error_message": "Please enter a valid input",
                "error_options": ("Okay", "Oops")
            }
            for value in optional_values.keys():
                if value not in node_info:
                    node_info[value] = optional_values[value]

            if speaking_rate is not None:
                node_info["content"] = "<prosody rate=\"{speaking_rate}\">".format(speaking_rate=speaking_rate) + \
                                       node_info["content"] + "</prosody> "

            if node_info["result_convert_from_str_fn"] == "save_tomorrow_checkin_datetime":
                node_info["result_convert_from_str_fn"] = self.next_day_checkin_datetime_from_str
            if node_info["result_convert_from_str_fn"] == "later_today_checkin_datetime":
                node_info["result_convert_from_str_fn"] = self.later_today_checkin_datetime

            if node_info["tests"] == "check reading id":
                node_info["tests"] = self.check_reading_id

            node = Node(
                name=node_name,
                transitions=node_info["transitions"],
                content=node_info["content"],
                options=node_info["options"],
                message_type=node_info["message_type"],
                args=node_info["args"],
                result_convert_from_str_fn=node_info["result_convert_from_str_fn"],
                result_db_key=node_info["result_db_key"],
                is_append_result=node_info["is_append_result"],
                tests=node_info["tests"],
                is_confirm=node_info["is_confirm"],
                error_message=node_info["error_message"],
                error_options=node_info["error_options"],
                text_populator=text_populator
            )
            nodes.append(node)

        return DirectedGraph(
            name=graph_name,
            nodes=nodes,
            start_node=start_node_name
        )

    def next_day_checkin_datetime_from_str(self, time_string):
        next_checkin_time = datetime.datetime.strptime(time_string, '%I:%M %p').time()
        current_datetime = datetime.datetime.now()
        current_day = current_datetime.day
        try:
            new_datetime = current_datetime.replace(
                day=current_day + 1,
                hour=next_checkin_time.hour,
                minute=next_checkin_time.minute
            )
        except ValueError:
            logging.info("Day out of range for current month, setting to first day of next month")
            current_month = current_datetime.month
            new_datetime = current_datetime.replace(
                month=current_month+1,
                day=1,
                hour=next_checkin_time.hour,
                minute=next_checkin_time.minute
            )
        return new_datetime

    def later_today_checkin_datetime(self, hours_as_str):
        try:
            hours = int(hours_as_str)
        except (TypeError, ValueError):
            logging.info("Could not set checkin for later today, defaulting to the current time tomorrow.")
            hours = 24
        current_hour = datetime.datetime.now().hour
        return datetime.datetime.now().replace(hour=current_hour+hours)

    def check_reading_id(self, reading_id):
        expected_id = self._statedb.get(DatabaseKeys.CURRENT_READING_ID)
        is_correct_id = reading_id == expected_id
        return is_correct_id

    @property
    def interactions(self):
        return self._interactions

    @property
    def possible_graphs(self):
        return self._possible_graphs
