#!/usr/bin/python3.8
import datetime
import logging
import schedule

from interaction_engine.engine import InteractionEngine
from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.planner import MessagerPlanner
from interaction_builder import InteractionBuilder

logging.basicConfig(level=logging.INFO)


class InteractionManager:

    class Interactions:
        DEMO_INTERACTION = "demo interaction"
        FIRST_INTERACTION = "first interaction"
        PROMPTED_INTERACTION = "prompted interaction"
        SCHEDULED_INTERACTION = "scheduled interaction"
        READING_EVALUATION = "reading evaluation"

        POSSIBLE_INTERACTIONS = [
            DEMO_INTERACTION,
            FIRST_INTERACTION,
            SCHEDULED_INTERACTION,
            PROMPTED_INTERACTION,
            READING_EVALUATION,
        ]

    def __init__(
            self,
            statedb,
            paramdb,
            interaction_builder,
            interface=None
    ):
        self._state_database = statedb
        self._param_database = paramdb

        self._interaction_builder = interaction_builder

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        self._planner = MessagerPlanner(self._interaction_builder.possible_graphs)

    def run_interaction_once(self, interaction_type):
        if interaction_type not in InteractionManager.Interactions.POSSIBLE_INTERACTIONS:
            raise ValueError("Not a valid interaction type")

        self.build_interaction(interaction_type, self._planner)
        engine = InteractionEngine(
            self._interface,
            self._planner,
            self._interaction_builder.possible_graphs
        )
        engine.run()

    def build_interaction(self, interaction_type, planner):
        build_interaction_dict = {
            InteractionManager.Interactions.DEMO_INTERACTION: self._build_demo_interaction,
            InteractionManager.Interactions.FIRST_INTERACTION: self._build_first_interaction,
            InteractionManager.Interactions.PROMPTED_INTERACTION: self._build_prompted_interaction,
            InteractionManager.Interactions.SCHEDULED_INTERACTION: self._build_scheduled_interaction,
            InteractionManager.Interactions.READING_EVALUATION: self._build_reading_evaluation,
        }

        return build_interaction_dict[interaction_type](planner)

    def _build_demo_interaction(self, planner):
        logging.info("Building first interaction")
        planner.insert(
            self._interaction_builder.interactions[InteractionBuilder.Graphs.DEMO_INTERACTION]
            # post_hook=self._set_last_interaction_time
        )
        return planner

    def _build_first_interaction(self, planner):
        logging.info("Building first interaction")
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.INTRODUCE_QT])
        # planner.insert(Interactions.FIRST_INTERACTION, post_hook=self._set_last_interaction_time)
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.FIRST_INTERACTION])
        return planner

    def _build_prompted_interaction(self, planner):
        logging.info("Building prompted interaction")
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.GREETING])
        # planner.insert(Interactions.PROMPTED_INTERACTION, post_hook=self._set_last_interaction_time)
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.PROMPTED_INTERACTION])
        return planner

    def _build_scheduled_interaction(self, planner):
        logging.info("Building scheduled interaction")
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.GREETING])
        # planner.insert(Interactions.SCHEDULED_INTERACTION, post_hook=self._set_last_interaction_time)
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.SCHEDULED_INTERACTION])
        return planner

    def _build_reading_evaluation(self, planner):
        logging.info("Building reading evaluation")
        # planner.insert(Interactions.READING_EVALUATION, post_hook=self._set_last_interaction_time)
        planner.insert(self._interaction_builder.interactions[InteractionBuilder.Graphs.READING_EVALUATION])
        return planner

    def _set_last_interaction_time(self):
        self._state_database.set("last interaction time", datetime.datetime.now())
