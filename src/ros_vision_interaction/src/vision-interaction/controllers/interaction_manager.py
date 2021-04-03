#!/usr/bin/python3.8
import datetime
import logging

from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.planner import MessagerPlanner
from interaction_builder import InteractionBuilder
from vision_project_tools.vision_engine import VisionInteractionEngine as InteractionEngine

logging.basicConfig(level=logging.INFO)


class Interactions:

    ASK_TO_DO_EVALUATION = "ask to do evaluation"
    EVALUATION = "evaluation"
    FIRST_INTERACTION = "first interaction"
    PROMPTED_CONTENT = "prompted content"
    PROMPTED_INTERACTION = "prompted interaction"
    SCHEDULED_INTERACTION = "scheduled interaction"
    TOO_MANY_CHECKINS = "too many checkins"

    POSSIBLE_INTERACTIONS = [
        ASK_TO_DO_EVALUATION,
        EVALUATION,
        FIRST_INTERACTION,
        PROMPTED_CONTENT,
        PROMPTED_INTERACTION,
        SCHEDULED_INTERACTION,
        TOO_MANY_CHECKINS
    ]


class InteractionManager:

    def __init__(
            self,
            statedb,
            interaction_builder,
            interface=None
    ):
        self._state_database = statedb

        self._interaction_builder = interaction_builder

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        self._current_node_name = None
        self._current_interaction_type = None

    def run_interaction_once(self, interaction_type):
        if interaction_type not in Interactions.POSSIBLE_INTERACTIONS:
            raise ValueError("Not a valid interaction type")

        planner = self.build_interaction(interaction_type)
        self.run_engine_once(planner)

    def run_engine_once(self, planner):
        engine = InteractionEngine(
            self._interface,
            planner,
            self._interaction_builder.possible_graphs
        )
        for node_name in engine.modified_run(planner):
            self._current_node_name = node_name

    def build_interaction(self, interaction_type):
        build_interaction_dict = {
            Interactions.ASK_TO_DO_EVALUATION: self._build_ask_to_do_scheduled,
            Interactions.FIRST_INTERACTION: self._build_first_interaction,
            Interactions.PROMPTED_INTERACTION: self._build_prompted_interaction,
            Interactions.SCHEDULED_INTERACTION: self._build_scheduled_interaction,
            Interactions.EVALUATION: self._build_reading_evaluation,
            Interactions.TOO_MANY_CHECKINS: self._build_too_many_checkins
        }

        return build_interaction_dict[interaction_type]()

    def _build_first_interaction(self):
        planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        logging.info("Building first interaction")
        planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.FIRST_CHECKIN],
            post_hook=self._set_vars_after_first_interaction
        )
        planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULE_NEXT_CHECKIN],
            post_hook=self._state_database.set("is interaction finished", True)
        )
        return planner

    def _build_ask_to_do_scheduled(self):
        planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        logging.info("Building ask to do scheduled")
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.GREETING])
        planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.ASK_TO_DO_SCHEDULED],
            post_hook=self._set_is_off_checkin
        )
        return planner

    def _build_prompted_interaction(self):
        planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        logging.info("Building prompted interaction")
        if self._state_database.get("is done evaluation today"):
            planner.insert(
                self._interaction_builder.interactions[InteractionBuilder.Graphs.GREETING]
            )
        planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_CHECKIN],
            post_hook=self._state_database.set("is interaction finished", True)
        )
        return planner

    def _build_scheduled_interaction(self):
        planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        logging.info("Building scheduled interaction")
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.GREETING])
        planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULED_CHECKIN],
            post_hook=self._set_vars_after_scheduled
        )
        return planner

    def _build_reading_evaluation(self):
        planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        logging.info("Building reading evaluation")
        planner.insert(
            plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.EVALUATION],
            post_hook=self._set_vars_after_evaluation
        )
        return planner

    def _build_too_many_checkins(self):
        planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        logging.info("Building checkin limit reminder")
        planner.insert(
            plan=self._interaction_builder.interactions[InteractionBuilder.Graphs.TOO_MANY_CHECKINS],
            post_hook=self._state_database.set("is interaction finished", True)
        )
        return planner

    def _set_vars_after_interaction(self):
        self._state_database.set("is interaction finished", True)

    def _set_vars_after_prompted(self):
        number_of_prompted_today = self._state_database.get("number of prompted today") + 1
        self._state_database.set("number of prompted today", number_of_prompted_today)
        self._state_database.set("is interaction finished", True)
        self._state_database.set("is run prompted content", False)
        self._state_database.set("is prompted by user", False)

    def _set_vars_after_first_interaction(self):
        self._state_database.set("first interaction datetime", datetime.datetime.now())
        self._state_database.set("is interaction finished", True)

    def _set_vars_after_evaluation(self):
        self._state_database.set("is done evaluation today", True)
        # also need to calculate and save reading speed
        self._state_database.set("is interaction finished", True)

    def _set_vars_after_scheduled(self):
        self._state_database.set("is prompted by user", False)

    def _set_is_off_checkin(self):
        if self._state_database.get("is off checkin") == "Yes":
            self._state_database.set("is off checkin", True)
            self._state_database.set("is run prompted content", False)
        else:
            self._state_database.set("is off checkin", False)
            self._state_database.set("is run prompted content", True)

    @property
    def current_node_name(self):
        return self._current_node_name
