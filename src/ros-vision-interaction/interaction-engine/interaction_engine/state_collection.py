#!/usr/bin/env python
import logging
import os

from interaction_engine.state import State
from interaction_engine.utils import json_load_byteified

from graphviz import Digraph

logging.basicConfig(level=logging.INFO)


class StateCollection(object):

    def __init__(
            self,
            name,
            init_state_name,
            states,
            exit_code="exit",
    ):
        self._states = {}
        if type(states) is dict:
            self._build_states_from_dict(states)
        else:
            for state in states:
                self.add_state(state)

        valid_state_names = [state_name for state_name in self._states.keys()]
        self._init_state = self._check_init_state(init_state_name, valid_state_names)
        self._exit_code = exit_code

        if type(name) is not str:
            raise TypeError("Graph name must be a string.")
        self._name = name

        self._check_for_unreachable_states()

        self._current_state = init_state_name
        self._is_running = True

    def _check_init_state(self, init_state, valid_state_names):
        if type(init_state) is not str:
            raise TypeError("Invalid initial state name.")
        if init_state not in valid_state_names:
            raise IOError("Initial state name '{}' does not correspond to a valid state.".format(init_state))
        return init_state

    def _check_next_states(self):
        for state in list(self._states.values()):
            for next_state in state.next_states:
                if next_state not in self._states.keys() and next_state != self._exit_code:
                    raise ValueError("Next states in {} do not correspond to a valid state.".format(state.name))

    def _check_for_unreachable_states(self):
        for state in list(self._states.values()):
            unreachable = state.name != self._init_state
            for other_state in list(self._states.values()):
                if state.name in other_state.next_states:
                    unreachable = False
            if unreachable:
                logging.warning("{} is unreachable.".format(state.name))

    def _build_states_from_dict(self, states_from_json):
        state_names = states_from_json.keys()
        for state_name in state_names:
            name = state_name
            message_type = states_from_json[state_name]["message type"]
            content = [content for content in states_from_json[state_name]["content"]]
            next_states = [next_state for next_state in states_from_json[state_name]["next states"]]
            transitions = states_from_json[state_name]["transitions"]
            # optional parameters; this check is to prevent key errors
            try:
                database_key_to_write = str(states_from_json[state_name]["database key to write"])
            except KeyError:
                database_key_to_write = None
            try:
                database_keys_to_read = [
                    str(key_to_read) for key_to_read in states_from_json[state_name]["database keys to read"]
                ]
            except KeyError:
                database_keys_to_read = None
            try:
                args = [
                    arg for arg in states_from_json[state_name]["args"]
                ]
            except KeyError:
                args = None

            new_state = State(
                name=name,
                message_type=message_type,
                content=content,
                next_states=next_states,
                transitions=transitions,
                database_key_to_write=database_key_to_write,
                database_keys_to_read=database_keys_to_read,
                args=args
            )

            self.add_state(new_state)

    @property
    def states(self):
        return self._states

    @property
    def name(self):
        return self._name

    @property
    def init_state(self):
        return self._init_state

    @property
    def current_state(self):
        return self._current_state

    @property
    def exit_code(self):
        return self._exit_code

    @property
    def is_running(self):
        return self._is_running

    @is_running.setter
    def is_running(self, new_value):
        if type(new_value) is not bool:
            raise TypeError("Must be a boolean.")
        self._is_running = new_value

    @current_state.setter
    def current_state(self, state_name):
        if type(state_name) is not str:
            raise TypeError("Must pass in state name.")
        if state_name not in list(self._states.keys()):
            raise ValueError("Not a valid state.")
        self._current_state = state_name

    def add_state(self, state):
        if type(state) is not State:
            raise TypeError("Invalid state.")
        if state.name in self._states:
            raise ValueError("\"{}\" already exists.".format(state.name))

        self._states[state.name] = state

    def transition(self, user_input=None):
        self._check_next_states()

        if len(self._states) == 1:
            self._is_running = False

        if self._is_running:
            next_state = self._states[self._current_state].get_next_state(user_input)
            self._current_state = next_state

            if next_state == "exit":
                self._is_running = False

    def reset(self):
        self._current_state = self._init_state

    def visualize(self, show_content=False):
        graph = Digraph(name=self._name, format="png")
        graph.attr("node", shape="box")
        node_names = [state_name for state_name in self._states.keys()]
        for name in node_names:
            if show_content:
                node_label = ""
                for string in self._states[name].message.content:
                    node_label += string + "\n"
            else:
                node_label = name
            graph.node(name, node_label)
        for state_name in self._states.keys():
            state = self._states[state_name]
            for i in range(len(state.transitions.keys())):
                option = state.message.options[i]
                graph.edge(state.name, state.transitions[i], label=option)

        graph.view(cleanup=True)


if __name__ == "__main__":
    interaction_json_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "../../scripts/demo-interaction/sar_demo_states.json"
    )
    with open(interaction_json_file) as f:
        interaction_setup_dict = json_load_byteified(f)
    states_dict_from_json = interaction_setup_dict["states"]
    state_collection = StateCollection(
        name="demo interaction",
        init_state_name="ask to chat",
        states=states_dict_from_json
    )

    state_collection.visualize()
