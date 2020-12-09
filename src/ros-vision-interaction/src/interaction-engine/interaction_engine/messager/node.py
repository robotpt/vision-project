from interaction_engine.messager.message import Message
from robotpt_common_utils import lists, math_tools


class Node:

    def __init__(
            self,
            name,
            transitions,
            message=None,
            content=None,
            options=None,
            message_type=None,
            args=None,
            result_convert_from_str_fn=str,
            result_db_key=None,
            is_append_result=False,
            tests=None,
            is_confirm=False,
            error_message="Please enter a valid input",
            error_options=('Okay', 'Oops'),
            text_populator=None,
            transition_fn=None,
    ):
        self._name = name

        if message is None:
            message = Message(
                content=content,
                options=options,
                message_type=message_type,
                args=args,
                result_convert_from_str_fn=result_convert_from_str_fn,
                result_db_key=result_db_key,
                is_append_result=is_append_result,
                tests=tests,
                is_confirm=is_confirm,
                error_message=error_message,
                error_options=error_options,
                text_populator=text_populator,
            )
        else:
            if options is not None:
                message.options = options
            if args is not None:
                message.args = args
        self._message = message

        transitions = lists.make_sure_is_iterable(transitions)
        if not self._is_transitions_valid(transitions, transition_fn):
            raise ValueError("Transitions should agree with message options")
        self._transitions = transitions

        if transition_fn is not None and not callable(transition_fn):
            raise ValueError('Transition function must be callable')
        self._transition_fn = transition_fn

    @property
    def name(self):
        return self._name

    @property
    def transitions(self):
        return self._transitions

    @property
    def message(self):
        return self._message

    def get_transition(self, user_input):
        if len(self._transitions) is 1:
            return self._transitions[0]
        elif self._transition_fn is None:
            return self._transitions[
                self.message.last_options.index(user_input)
            ]
        else:
            idx = self._transition_fn(user_input)
            if not math_tools.is_int(idx):
                raise IOError("Transition function must return an index")
            return self._transitions[idx]

    def _is_transitions_valid(self, transitions, transitions_fn=None):

        is_transition_fn = transitions_fn is not None
        if is_transition_fn and not callable(transitions_fn):
            raise IOError("'transitions_fn' must be callable")

        num_transitions = len(transitions)
        num_options = len(self._message.options)

        if self._message.message_type is Message.Type.MULTIPLE_CHOICE:
            return Node._is_valid_multiple_choice(is_transition_fn, num_options, num_transitions)
        elif self._message.message_type is Message.Type.DIRECT_INPUT:
            return Node._is_valid_direct_entry(is_transition_fn, num_options, num_transitions)
        else:
            return NotImplementedError("Invalid message type")

    @staticmethod
    def _is_valid_direct_entry(is_transition_fn, num_options, num_transitions):
        is_one_option = num_options == 1
        if is_transition_fn:
            is_one_or_more_transitions = num_transitions >= 1
            return is_one_option and is_one_or_more_transitions
        else:
            is_one_transition = num_transitions == 1
            return is_one_option and is_one_transition

    @staticmethod
    def _is_valid_multiple_choice(is_transition_fn, num_options, num_transitions):
        if is_transition_fn:
            return False
        else:
            is_transition_for_each_option = num_options == num_transitions
            is_one_transition_for_all_options = (
                    num_transitions == 1 and num_options >= 1
            )
            return is_transition_for_each_option or is_one_transition_for_all_options
