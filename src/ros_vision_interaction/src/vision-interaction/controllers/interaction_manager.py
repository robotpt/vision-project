#!/usr/bin/python3.8
import datetime
import logging
import vision_project_tools.reading_task_tools as reading_task_tools

from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.planner import MessagerPlanner
from interaction_builder import InteractionBuilder
from vision_project_tools import increment_db_value
from vision_project_tools.constants import Interactions, DatabaseKeys
from vision_project_tools.reading_task_tools import TaskDataKeys, Tasks
from vision_project_tools.vision_engine import VisionInteractionEngine as InteractionEngine

logging.basicConfig(level=logging.INFO)


class InteractionManager:

    def __init__(
            self,
            statedb,
            interaction_builder,
            interface=None,
            max_num_of_perseverance_readings=2,
            max_num_of_spot_reading_attempts=1
    ):
        self._state_database = statedb

        self._interaction_builder = interaction_builder

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        self._current_node_name = None
        self._current_interaction_type = None

        self._planner = MessagerPlanner(self._interaction_builder.possible_graphs)

        self._max_num_of_perseverance_readings = max_num_of_perseverance_readings
        self._max_num_of_spot_reading_attempts = max_num_of_spot_reading_attempts
        self._num_of_days_to_prompt_goal_setting = 3
        self._spot_reading_attempts = 0
        self._spot_reading_index = 0
        self._spot_reading_perseverance_index = 0

    def run_interaction_once(self, interaction_type):
        if interaction_type not in Interactions.POSSIBLE_INTERACTIONS:
            raise ValueError("Not a valid interaction type")

        self.build_interaction(interaction_type)
        for node_name in self.run_engine_once():
            yield node_name

    def run_engine_once(self):
        engine = InteractionEngine(
            self._interface,
            self._planner,
            self._interaction_builder.possible_graphs
        )
        for node_name in engine.modified_run(self._planner):
            yield node_name

    def build_interaction(self, interaction_type):
        self._planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        if interaction_type == Interactions.ASK_TO_DO_SCHEDULED:
            self._build_ask_to_do_scheduled()
        elif interaction_type == Interactions.FIRST_INTERACTION:
            self._build_first_interaction()
        elif interaction_type == Interactions.PERSEVERANCE:
            self._state_database.set(DatabaseKeys.IS_START_PERSEVERANCE, True)
            self._state_database.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
            self._build_perseverance()
        elif interaction_type == Interactions.PROMPTED_INTERACTION:
            self._build_prompted_interaction()
        elif interaction_type == Interactions.SCHEDULED_INTERACTION:
            self._build_scheduled_interaction()
        elif interaction_type == Interactions.TOO_MANY_PROMPTED:
            self._build_too_many_prompted()
        elif interaction_type == Interactions.EVALUATION:
            self._build_evaluation()
        else:
            raise ValueError("Not a valid interaction type")

        return self._planner

    def _build_evaluation(self):
        if self._state_database.get(DatabaseKeys.FIRST_INTERACTION_DATETIME).date() < datetime.datetime.now().date():
            if self._state_database.get(DatabaseKeys.VIDEO_TO_PLAY):
                video_type = self._state_database.get(DatabaseKeys.VIDEO_TO_PLAY)
                video_dictionary = self._state_database.get(DatabaseKeys.FEEDBACK_VIDEOS)
                video_index = self._get_video_index(video_type)
                self._state_database.set(DatabaseKeys.VIDEO_TO_PLAY, video_dictionary[video_type])
                self._state_database.set(DatabaseKeys.VIDEO_INTRO_INDEX, video_index)
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.FEEDBACK_VIDEO],
                    post_hook=self._set_vars_after_interaction
                )
            else:
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.NO_FEEDBACK_VIDEO],
                    post_hook=self._set_vars_after_interaction
                )
            if self._state_database.get(DatabaseKeys.LAST_SCORE) is not None and \
                    self._state_database.get(DatabaseKeys.BEST_SCORE) is not None:
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.LAST_EVALUATION],
                    post_hook=self._set_vars_after_interaction
                )

        self._planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.INTRODUCE_EVALUATION],
            post_hook=self._set_vars_after_interaction
        )

        task_type = reading_task_tools.get_current_reading_task_type(self._state_database)

        if task_type == Tasks.IREST:
            increment_db_value(self._state_database, DatabaseKeys.IREST_READING_INDEX)
            self._set_new_task_info()

        if task_type == reading_task_tools.Tasks.SPOT_READING:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.SPOT_READING_EVAL],
                post_hook=self._set_vars_after_spot_reading_eval
            )
        else:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.EVALUATION],
                post_hook=self._set_vars_after_evaluation
            )

    def _get_video_index(self, video_name):
        videos = {
            "distance 4x": 3,
            "distance 6x": 3,
            "light": 4,
            "parallel": 3,
            "steady": 3,
            "upside down": 1,
            "no video": 5
        }
        return videos[video_name]

    def _build_ask_to_do_scheduled(self):
        logging.info("Building ask to do scheduled")
        self._planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_TO_DO_SCHEDULED],
            post_hook=self._set_vars_after_ask_to_do_scheduled,
        )
        return self._planner

    def _build_first_interaction(self):
        logging.info("Building first interaction")
        self._planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.INTRODUCE_QT],
            post_hook=self._set_vars_after_first_interaction
        )
        return self._planner

    def _build_perseverance(self):
        self._planner.insert(
            plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.INTRODUCE_EVALUATION],
            post_hook=self._set_vars_after_interaction
        )

        task_type = reading_task_tools.get_current_reading_task_type(self._state_database)

        if task_type == Tasks.IREST:
            increment_db_value(self._state_database, DatabaseKeys.IREST_READING_INDEX)
            self._set_new_task_info()

        if task_type == reading_task_tools.Tasks.SPOT_READING:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.SPOT_READING_EVAL],
                post_hook=self._set_vars_after_spot_reading_eval
            )
        else:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.PERSEVERANCE],
                post_hook=self._set_vars_after_perseverance
            )

    def _build_prompted_interaction(self):
        logging.info("Building prompted interaction")
        self._planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_ASK_TO_CHAT],
            post_hook=self._set_vars_after_prompted_ask_to_chat
        )
        return self._planner

    def _build_scheduled_interaction(self):
        logging.info("Building scheduled interaction")
        self._planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULED_ASK_TO_CHAT],
            post_hook=self._set_vars_after_scheduled_ask_to_chat
        )
        return self._planner

    def _build_too_many_prompted(self):
        logging.info("Building checkin limit reminder")
        self._planner.insert(
            plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.TOO_MANY_PROMPTED],
            post_hook=self._set_vars_after_too_many_prompted
        )
        return self._planner

    def _set_vars_after_ask_to_do_scheduled(self):
        if self._state_database.get(DatabaseKeys.IS_OFF_CHECKIN) == "Yes":
            self._state_database.set(DatabaseKeys.IS_OFF_CHECKIN, None)
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULED_CHECKIN],
                post_hook=self._set_vars_after_interaction
            )
        else:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_CHECKIN],
                post_hook=self._set_vars_after_prompted
            )

    def _set_vars_after_ask_for_eval_during_scheduled(self):
        if self._state_database.get(DatabaseKeys.IS_DO_EVAL_DURING_PROMPTED) == "Yes":
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_CHECKIN],
                post_hook=self._set_vars_after_prompted
            )
        else:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULED_CHECKIN],
                post_hook=self._set_vars_after_interaction
            )

    def _set_vars_after_scheduled_ask_for_eval(self):
        if self._state_database.get(DatabaseKeys.IS_DO_EVALUATION) == "Yes":
            self._state_database.set(DatabaseKeys.IS_DO_EVALUATION, None)
            self._build_evaluation()

    def _set_vars_after_spot_reading_eval(self):
        task_id = self._state_database.get(DatabaseKeys.CURRENT_READING_ID)
        is_doing_perseverance = self._state_database.get(DatabaseKeys.IS_DONE_EVAL_TODAY)
        reading_task_tools.set_reading_task_value(
            self._state_database,
            task_id,
            TaskDataKeys.IS_SCHEDULED,
            not is_doing_perseverance
        )

        answers = reading_task_tools.get_reading_task_data_value(
            self._state_database,
            task_id,
            TaskDataKeys.ANSWER
        )
        num_of_spot_reading = len(answers)

        answer = self._state_database.get(DatabaseKeys.SPOT_READING_ANSWER)
        is_correct = answer == answers[self._spot_reading_index]
        retry = self._spot_reading_attempts < self._max_num_of_spot_reading_attempts and \
            not is_correct and answer != "Unable to read"
        if retry:
            self._spot_reading_attempts += 1
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.RETRY_SPOT_READING],
                post_hook=self._set_vars_after_spot_reading_eval
            )

        else:
            is_finished = self._spot_reading_index == num_of_spot_reading - 1
            if is_finished:
                self._spot_reading_index = 0
                self._spot_reading_attempts = 0
                if is_doing_perseverance:
                    self._spot_reading_perseverance_index += 1
                    self._set_vars_after_perseverance()
                else:
                    self._planner.insert(
                        self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_EVALUATION],
                        post_hook=self._set_vars_after_post_eval
                    )
            else:
                self._spot_reading_index += 1
                self._spot_reading_attempts = 0
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.SPOT_READING_EVAL],
                    post_hook=self._set_vars_after_spot_reading_eval
                )

    def _set_vars_after_post_eval(self):
        self._state_database.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
        increment_db_value(self._state_database, DatabaseKeys.READING_EVAL_INDEX)
        self._spot_reading_index = 0
        new_rating = self._state_database.get(DatabaseKeys.FEELINGS_INDEX)
        grit_feedback_index = 0

        try:
            # self_reports sometimes = None, even though it's set as []
            len(self._state_database.get(DatabaseKeys.SELF_REPORTS))
        except TypeError:
            self._state_database.set(DatabaseKeys.SELF_REPORTS, [])
        self_ratings = self._state_database.get(DatabaseKeys.SELF_REPORTS)
        if len(self_ratings) > 0:
            average_self_rating = sum(self_ratings) / len(self_ratings)
            if new_rating < average_self_rating:
                grit_feedback_index = 1  # BETTER RATING
            if new_rating > average_self_rating:
                grit_feedback_index = 2  # WORSE RATING

        self._state_database.set(DatabaseKeys.GRIT_FEEDBACK_INDEX, grit_feedback_index)
        self._state_database.set(DatabaseKeys.SELF_REPORTS, self_ratings.append(new_rating))

        self._planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_TO_DO_PERSEVERANCE],
            post_hook=self._set_vars_after_ask_to_do_perseverance
        )

        self._set_vars_after_interaction()

    def _set_vars_after_prompted_ask_to_chat(self):
        if self._state_database.get(DatabaseKeys.GOOD_TO_CHAT) == "Yes":
            self._state_database.set(DatabaseKeys.GOOD_TO_CHAT, None)
            if not self._state_database.get(DatabaseKeys.IS_DONE_EVAL_TODAY):
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_FOR_EVAL_DURING_PROMPTED],
                    post_hook=self._set_vars_after_ask_for_eval_during_scheduled
                )
            else:
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_CHECKIN],
                    post_hook=self._set_vars_after_prompted
                )
        else:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_PLAN_NEXT_CHECKIN],
                post_hook=self._set_vars_after_interaction
            )

    def _set_vars_after_scheduled_ask_to_chat(self):
        if self._state_database.get(DatabaseKeys.GOOD_TO_CHAT) == "Yes":
            self._state_database.set(DatabaseKeys.GOOD_TO_CHAT, None)
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULED_CHECKIN],
                post_hook=self._set_vars_after_interaction
            )
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_TO_DO_EVALUATION],
                post_hook=self._set_vars_after_scheduled_ask_for_eval
            )
        else:
            self._state_database.set(DatabaseKeys.GOOD_TO_CHAT, None)
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_NEXT_CHECKIN],
                post_hook=self._set_vars_after_interaction
            )

    def _set_vars_after_ask_to_do_perseverance(self):
        if self._state_database.get(DatabaseKeys.IS_START_PERSEVERANCE) == "Yes":
            self._state_database.set(DatabaseKeys.IS_START_PERSEVERANCE, None)
            self._build_perseverance()
        else:
            if self._is_do_mindfulness():
                index = self._state_database.get(DatabaseKeys.MINDFULNESS_INDEX)
                if index == 0:
                    mindfulness = InteractionBuilder.Graphs.MINDFULNESS_BODY_SCAN
                elif index == 1:
                    mindfulness = InteractionBuilder.Graphs.MINDFULNESS_BREATHING
                else:
                    mindfulness = InteractionBuilder.Graphs.MINDFULNESS_BODY_SCAN
                self._planner.insert(
                    plan=self._interaction_builder.interactions[mindfulness],
                    post_hook=self._set_vars_after_mindfulness
                )
            if self._is_do_goal_setting():
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.GOAL_SETTING],
                    post_hook=self._set_vars_after_goal_setting
                )
            if self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT) >= 3:
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.REMINDER_FOR_PROMPTED],
                    post_hook=self._set_vars_after_interaction
                )

            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_CHECKIN_TOMORROW],
                post_hook=self._set_vars_after_interaction
            )

    def _set_vars_after_evaluation(self):
        self._state_database.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
        task_type = reading_task_tools.get_current_reading_task_type(self._state_database)
        if task_type == reading_task_tools.Tasks.IREST:
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_IREST],
                post_hook=self._set_vars_after_interaction
            )
        if task_type == reading_task_tools.Tasks.SRT:
            if int(self._state_database.get(DatabaseKeys.CURRENT_READING_ID)) % 3 == 0:
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_SSRT],
                    post_hook=self._set_vars_after_interaction
                )
                increment_db_value(self._state_database, DatabaseKeys.POST_SRT_INDEX)
            increment_db_value(self._state_database, DatabaseKeys.SRT_READING_INDEX)

        self._set_new_task_info()

        self._planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_EVALUATION],
            post_hook=self._set_vars_after_post_eval
        )

    def _set_vars_after_interaction(self):
        self._state_database.set(DatabaseKeys.IS_PROMPTED_BY_USER, False)
        self._state_database.set(DatabaseKeys.IS_INTERACTION_FINISHED, True)
        self._state_database.set(DatabaseKeys.LAST_INTERACTION_DATETIME, datetime.datetime.now())

    def _set_vars_after_first_interaction(self):
        if self._state_database.get(DatabaseKeys.GOOD_TO_CHAT) == "Yes":
            self._state_database.set(DatabaseKeys.GOOD_TO_CHAT, None)
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULED_CHECKIN],
                post_hook=self._set_vars_after_interaction
            )
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_TO_DO_EVALUATION],
                post_hook=self._set_vars_after_scheduled_ask_for_eval
            )
        else:
            self._state_database.set(DatabaseKeys.GOOD_TO_CHAT, None)
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_NEXT_CHECKIN],
                post_hook=self._set_vars_after_interaction
            )
        self._state_database.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime.now())

    def _set_vars_after_goal_setting(self):
        self._state_database.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING, 0)
        new_rating = {
            datetime.datetime.now(): {
                "type": "goal setting",
                "rating": self._state_database.get(DatabaseKeys.GOAL_RATING)
            }
        }
        ratings = self._state_database.get(DatabaseKeys.ACT_RATINGS)
        self._state_database.set(DatabaseKeys.ACT_RATINGS, ratings.update(new_rating))
        self._set_vars_after_interaction()

    def _set_vars_after_mindfulness(self):
        index = self._state_database.get(DatabaseKeys.MINDFULNESS_INDEX)
        self._state_database.set(DatabaseKeys.MINDFULNESS_INDEX, (index + 1) % 3)
        self._state_database.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS, 0)
        new_rating = {
            datetime.datetime.now(): {
                "type": "mindfulness",
                "rating": self._state_database.get(DatabaseKeys.MINDFULNESS_RATING)
            }
        }
        ratings = self._state_database.get(DatabaseKeys.ACT_RATINGS)
        self._state_database.set(DatabaseKeys.ACT_RATINGS, ratings.update(new_rating))
        self._set_vars_after_interaction()

    def _set_vars_after_perseverance(self):
        increment_db_value(self._state_database, DatabaseKeys.PERSEVERANCE_COUNTER)
        perseverance_counter = self._state_database.get(DatabaseKeys.PERSEVERANCE_COUNTER)

        task_type = reading_task_tools.get_current_reading_task_type(self._state_database)

        if perseverance_counter >= self._max_num_of_perseverance_readings:
            self._state_database.set(DatabaseKeys.PERSEVERANCE_COUNTER, 0)

            if task_type == reading_task_tools.Tasks.SRT:
                if int(self._state_database.get(DatabaseKeys.CURRENT_READING_ID)) % 3 == 0:
                    self._planner.insert(
                        plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_SSRT],
                        post_hook=self._set_vars_after_interaction
                    )
                    increment_db_value(self._state_database, DatabaseKeys.SRT_READING_INDEX)
                increment_db_value(self._state_database, DatabaseKeys.POST_SRT_INDEX)

            if task_type == Tasks.SPOT_READING:
                self._spot_reading_attempts = 0
                self._spot_reading_perseverance_index = 0

            if task_type == reading_task_tools.Tasks.IREST:
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_IREST],
                    post_hook=self._set_vars_after_interaction
                )

            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.REWARD],
                post_hook=self._set_vars_after_interaction
            )
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_CHECKIN_TOMORROW]
            )
        else:
            if task_type == Tasks.SRT:
                increment_db_value(self._state_database, DatabaseKeys.SRT_READING_INDEX)
            if task_type == Tasks.IREST:
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_IREST],
                    post_hook=self._set_vars_after_interaction
                )
            self._set_new_task_info()

            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.CONTINUE_PERSEVERANCE],
                post_hook=self._set_vars_after_continue_perseverance
            )

    def _set_vars_after_continue_perseverance(self):
        task_type = reading_task_tools.get_current_reading_task_type(self._state_database)
        if task_type == Tasks.SRT:
            if int(self._state_database.get(DatabaseKeys.CURRENT_READING_ID)) % 3 == 0:
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_SSRT],
                    post_hook=self._set_vars_after_interaction
                )
        if self._state_database.get(DatabaseKeys.IS_CONTINUE_PERSEVERANCE) == "Continue":
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.INTRODUCE_EVALUATION],
                post_hook=self._set_vars_after_interaction
            )
            if task_type == Tasks.IREST:
                increment_db_value(self._state_database, DatabaseKeys.IREST_READING_INDEX)
                self._set_new_task_info()
            if task_type == reading_task_tools.Tasks.SPOT_READING:
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.SPOT_READING_EVAL],
                    post_hook=self._set_vars_after_spot_reading_eval
                )
            else:
                self._set_new_task_info()
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.PERSEVERANCE],
                    post_hook=self._set_vars_after_perseverance
                )
        else:
            self._state_database.set(DatabaseKeys.PERSEVERANCE_COUNTER, 0)
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.REWARD]
            )
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_CHECKIN_TOMORROW],
                post_hook=self._set_vars_after_interaction
            )

    def _set_new_task_info(self):
        task_id = reading_task_tools.get_new_day_reading_task(self._state_database)
        self._state_database.set(DatabaseKeys.CURRENT_READING_ID, task_id)
        task_color = reading_task_tools.get_reading_task_data_value(
            self._state_database, task_id, TaskDataKeys.COLOR
        )
        self._state_database.set(DatabaseKeys.CURRENT_READING_COLOR, task_color)

    def _set_vars_after_prompted(self):
        increment_db_value(self._state_database, DatabaseKeys.NUM_OF_PROMPTED_TODAY)
        if self._state_database.get(DatabaseKeys.DID_USE_MAGNIFIER) == "No":
            increment_db_value(self._state_database, DatabaseKeys.DAYS_WITHOUT_MAGNIFIER)
        if self._state_database.get(DatabaseKeys.DID_USE_MAGNIFIER) == "Yes":
            self._state_database.set(DatabaseKeys.DAYS_WITHOUT_MAGNIFIER, 0)

        if self._state_database.get(DatabaseKeys.DAYS_WITHOUT_MAGNIFIER) > 3:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_IF_GIVEN_UP],
                post_hook=self._set_vars_after_ask_if_given_up
            )

        self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.STORIES_AND_JOKES],
                post_hook=self._set_vars_after_interaction
        )
        self._state_database.set(DatabaseKeys.IS_PROMPTED_BY_USER, False)
        self._state_database.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
        self._set_vars_after_interaction()

    def _set_vars_after_ask_if_given_up(self):
        if self._state_database.get(DatabaseKeys.ASK_IF_GIVEN_UP) == "Yes":
            is_given_up = self._state_database.get(DatabaseKeys.IS_GIVEN_UP)
            is_given_up.append(datetime.datetime.now())
            self._state_database.set(DatabaseKeys.IS_GIVEN_UP, is_given_up)
        self._set_vars_after_interaction()

    def _set_vars_after_too_many_prompted(self):
        self._set_vars_after_interaction()
        self._state_database.set(DatabaseKeys.IS_PROMPTED_BY_USER, False)

    def _is_do_mindfulness(self):
        is_do_mindfulness = False
        if self._days_since_first_interaction() >= 3:
            num_of_values = 3
            average_self_report = sum(
                self._state_database.get(DatabaseKeys.SELF_REPORTS)[-num_of_values:]) / num_of_values
            is_do_mindfulness = self._state_database.get(DatabaseKeys.FEELINGS_INDEX) < average_self_report
        return is_do_mindfulness

    # Long-term deployment version
    # def _is_do_mindfulness(self):
    #     last_5_scores = self._state_database.get(DatabaseKeys.LAST_5_EVAL_SCORES)
    #     if len(last_5_scores) > 0:
    #         average_eval_score = sum(last_5_scores) / len(last_5_scores)
    #     else:
    #         average_eval_score = 0
    #     return self._days_since_first_interaction() >= 7 \
    #         and self._state_database.get(DatabaseKeys.FEELINGS_INDEX) <= 3 \
    #         and self._state_database.get(DatabaseKeys.CURRENT_EVAL_SCORE) < average_eval_score

    def _is_do_goal_setting(self):
        return self._days_since_first_interaction() >= 3 \
               and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_EVAL) >= 1 \
               and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING) >= 3 \
               and self._state_database.get(
            DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT) >= self._num_of_days_to_prompt_goal_setting \
               and self._state_database.get(
            DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE) >= self._num_of_days_to_prompt_goal_setting

    # Long-term deployment version
    # def _is_do_goal_setting(self):
    #     last_5_scores = self._state_database.get(DatabaseKeys.LAST_5_EVAL_SCORES)
    #     if len(last_5_scores) > 0:
    #         average_eval_score = sum(last_5_scores)/len(last_5_scores)
    #     else:
    #         average_eval_score = 0
    #     return self._days_since_first_interaction() >= 7 \
    #         and self._state_database.get(DatabaseKeys.FEELINGS_INDEX) <= 3 \
    #         and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING) >= 7 \
    #         and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT) >= self._num_of_days_to_prompt_goal_setting \
    #         and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE) >= self._num_of_days_to_prompt_goal_setting \
    #         and self._state_database.get(DatabaseKeys.CURRENT_EVAL_SCORE) < average_eval_score

    def _days_since_first_interaction(self):
        first_interaction_datetime = self._state_database.get(DatabaseKeys.FIRST_INTERACTION_DATETIME)
        if first_interaction_datetime is None:
            num_of_days = 0
        else:
            current_date = datetime.datetime.now().date()
            num_of_days = (current_date - self._state_database.get(DatabaseKeys.FIRST_INTERACTION_DATETIME).date()).days
        return num_of_days

    @property
    def current_node_name(self):
        return self._current_node_name
