#!/usr/bin/python3.8
import datetime
import logging

from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.planner import MessagerPlanner
from interaction_builder import InteractionBuilder
from vision_project_tools.constants import Interactions, DatabaseKeys
from vision_project_tools.vision_engine import VisionInteractionEngine as InteractionEngine

logging.basicConfig(level=logging.INFO)


class InteractionManager:

    def __init__(
            self,
            statedb,
            interaction_builder,
            interface=None,
            max_num_of_perseverance_readings=5
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
        self._num_of_days_to_prompt_goal_setting = 3

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
        elif interaction_type == Interactions.PROMPTED_INTERACTION:
            self._build_prompted_interaction()
        elif interaction_type == Interactions.SCHEDULED_INTERACTION:
            self._build_scheduled_interaction()
        elif interaction_type == Interactions.TOO_MANY_PROMPTED:
            self._build_too_many_prompted()
        else:
            raise ValueError("Not a valid interaction type")

        return self._planner

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
            if self._state_database.get(DatabaseKeys.VIDEO_TO_PLAY):
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.FEEDBACK_VIDEO],
                )
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.EVALUATION],
                post_hook=self._set_vars_after_evaluation
            )
        else:
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_ASK_TO_CHAT],
                post_hook=self._set_vars_after_prompted_ask_to_chat
            )

    def _set_vars_after_scheduled_ask_for_eval(self):
        if self._state_database.get(DatabaseKeys.IS_DO_EVALUATION) == "Yes":
            self._state_database.set(DatabaseKeys.IS_DO_EVALUATION, None)
            if self._state_database.get(DatabaseKeys.VIDEO_TO_PLAY):
                self._planner.insert(
                    self._interaction_builder.interactions[InteractionBuilder.Graphs.FEEDBACK_VIDEO],
                )
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.INTRODUCE_EVALUATION],
                post_hook=self._set_vars_after_interaction
            )
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.EVALUATION],
                post_hook=self._set_vars_after_evaluation
            )
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.POST_EVALUATION],
                post_hook=self._set_vars_after_interaction
            )
            self._planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_TO_DO_PERSEVERANCE],
                post_hook=self._set_vars_after_ask_to_do_perseverance
            )

    def _set_vars_after_prompted_ask_to_chat(self):
        if self._state_database.get(DatabaseKeys.GOOD_TO_CHAT) == "Yes":
            self._state_database.set(DatabaseKeys.GOOD_TO_CHAT, None)
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
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PERSEVERANCE],
                post_hook=self._set_vars_after_perseverance
            )
        else:
            if self._is_do_mindfulness():
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.MINDFULNESS],
                    post_hook=self._set_vars_after_mindfulness
                )
            if self._is_do_goal_setting():
                self._planner.insert(
                    plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.GOAL_SETTING],
                    post_hook=self._set_vars_after_goal_setting
                )
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_CHECKIN_TOMORROW],
                post_hook=self._set_vars_after_interaction
            )

    def _set_vars_after_evaluation(self):
        self._state_database.set(DatabaseKeys.IS_DONE_EVAL_TODAY, True)
        eval_index = self._state_database.get(DatabaseKeys.READING_EVAL_INDEX)
        self._state_database.set(DatabaseKeys.READING_EVAL_INDEX, eval_index + 1)
        # also need to calculate and save reading speed
        self._set_vars_after_interaction()

    def _set_vars_after_interaction(self):
        self._state_database.set(DatabaseKeys.IS_PROMPTED_BY_USER, False)
        self._state_database.set(DatabaseKeys.IS_INTERACTION_FINISHED, True)
        self._state_database.set(DatabaseKeys.LAST_INTERACTION_DATETIME, datetime.datetime.now())

    def _set_vars_after_first_interaction(self):
        self._state_database.set(DatabaseKeys.FIRST_INTERACTION_DATETIME, datetime.datetime.now())
        self._set_vars_after_interaction()

    def _set_vars_after_goal_setting(self):
        self._state_database.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING, 0)
        self._set_vars_after_interaction()

    def _set_vars_after_mindfulness(self):
        self._state_database.set(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_MINDFULNESS, 0)
        self._set_vars_after_interaction()

    def _set_vars_after_perseverance(self):
        perseverance_counter = self._state_database.get(DatabaseKeys.PERSEVERANCE_COUNTER) + 1
        self._state_database.set(DatabaseKeys.PERSEVERANCE_COUNTER, perseverance_counter)
        if perseverance_counter >= self._max_num_of_perseverance_readings:
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.REWARD]
            )
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_CHECKIN_TOMORROW]
            )
        else:
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.CONTINUE_PERSEVERANCE],
                post_hook=self._set_vars_after_continue_perseverance
            )

    def _set_vars_after_continue_perseverance(self):
        if self._state_database.get(DatabaseKeys.IS_CONTINUE_PERSEVERANCE) == "Continue":
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PERSEVERANCE],
                post_hook=self._set_vars_after_perseverance
            )
        else:
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.REWARD]
            )
            self._planner.insert(
                plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.PLAN_CHECKIN_TOMORROW],
                post_hook=self._set_vars_after_interaction
            )

    def _set_vars_after_prompted(self):
        num_of_prompted_today = self._state_database.get(DatabaseKeys.NUM_OF_PROMPTED_TODAY) + 1
        self._state_database.set(DatabaseKeys.NUM_OF_PROMPTED_TODAY, num_of_prompted_today)
        self._state_database.set(DatabaseKeys.IS_PROMPTED_BY_USER, False)
        self._state_database.set(DatabaseKeys.IS_DONE_PROMPTED_TODAY, True)
        self._set_vars_after_interaction()

    def _set_vars_after_too_many_prompted(self):
        self._set_vars_after_interaction()
        self._state_database.set(DatabaseKeys.IS_PROMPTED_BY_USER, False)

    def _is_do_mindfulness(self):
        last_5_scores = self._state_database.get(DatabaseKeys.LAST_5_EVAL_SCORES)
        if len(last_5_scores) > 0:
            average_eval_score = sum(last_5_scores) / len(last_5_scores)
        else:
            average_eval_score = 0
        return self._days_since_first_interaction() >= 7 \
            and self._state_database.get(DatabaseKeys.FEELINGS_INDEX) <= 3 \
            and self._state_database.get(DatabaseKeys.CURRENT_EVAL_SCORE) < average_eval_score

    def _is_do_goal_setting(self):
        last_5_scores = self._state_database.get(DatabaseKeys.LAST_5_EVAL_SCORES)
        if len(last_5_scores) > 0:
            average_eval_score = sum(last_5_scores)/len(last_5_scores)
        else:
            average_eval_score = 0
        return self._days_since_first_interaction() >= 7 \
            and self._state_database.get(DatabaseKeys.FEELINGS_INDEX) <= 3 \
            and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_GOAL_SETTING) >= 7 \
            and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PROMPT) >= self._num_of_days_to_prompt_goal_setting \
            and self._state_database.get(DatabaseKeys.NUM_OF_DAYS_SINCE_LAST_PERSEVERANCE) >= self._num_of_days_to_prompt_goal_setting \
            and self._state_database.get(DatabaseKeys.CURRENT_EVAL_SCORE) < average_eval_score

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
