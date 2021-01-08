#!/usr/bin/python3.8
import datetime
import json
import logging
import os
import rospy
import schedule

from interaction_engine.engine import InteractionEngine
from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.planner import MessagerPlanner

logging.basicConfig(level=logging.INFO)


class InteractionManager:
    class Interactions:
        FIRST_INTERACTION = "first interaction"
        PROMPTED_INTERACTION = "prompted interaction"
        SCHEDULED_INTERACTION = "scheduled interaction"
        READING_EVALUATION = "reading evaluation"

        POSSIBLE_INTERACTIONS = [
            FIRST_INTERACTION,
            SCHEDULED_INTERACTION,
            PROMPTED_INTERACTION,
            READING_EVALUATION,
        ]

    def __init__(
            self,
            mongodb_statedb,
            text_populator=None,
            interface=None
    ):
        self._state_database = mongodb_statedb

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        self._planner = None

        self._text_populator = text_populator

    def run_interaction_once(self, interaction_type):
        if interaction_type not in InteractionManager.Interactions.POSSIBLE_INTERACTIONS:
            raise ValueError("Not a valid interaction type")

        self.build_interaction(interaction_type, self._planner)
        engine = InteractionEngine(
            self._interface,
            self._planner,
            list(self._graphs_dict.values())
        )
        engine.run()

    def build_interaction(self, interaction_type, planner):
        build_interaction_dict = {
            InteractionManager.Interactions.FIRST_INTERACTION: self._build_first_interaction,
            InteractionManager.Interactions.PROMPTED_INTERACTION: self._build_prompted_interaction,
            InteractionManager.Interactions.SCHEDULED_INTERACTION: self._build_scheduled_interaction,
            InteractionManager.Interactions.READING_EVALUATION:
                lambda planner: planner.insert(self._graphs_dict["reading evaluation"]),
        }

        return build_interaction_dict[interaction_type](planner)

    def _build_first_interaction(self, planner):
        rospy.loginfo("Building first interaction")
        planner.insert(self._graphs_dict["introduce QT"])
        planner.insert(self._graphs_dict["first interaction"])
        return planner

    def _build_prompted_interaction(self, planner):
        rospy.loginfo("Building prompted interaction")
        planner.insert(self._graphs_dict["greeting"])
        planner.insert(self._graphs_dict["prompted interaction"])
        return planner

    def _build_scheduled_interaction(self, planner):
        rospy.loginfo("Building scheduled interaction")
        planner.insert(self._graphs_dict["greeting"])
        planner.insert(self._graphs_dict["scheduled interaction"])
        return planner

