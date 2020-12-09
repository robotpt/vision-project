from interaction_engine.messager import DirectedGraph, Node, Message
from robotpt_common_utils import lists


def most_recent_options_graph(
        name,
        content,
        options,
        max_num_options=None,
        new_entry_text_choice='Something else',
        new_entry_message=None,
        new_entry_options='Okay',
        new_entry_error_message=None,
        result_convert_from_str_fn=str,
        tests=None,
        save_db_key=None,
        text_populator=None,
        is_append_result=True,
):
    if not callable(options):
        raise ValueError("Options should be callable")
    if new_entry_message is None:
        new_entry_message = content
    return DirectedGraph(
        name=name,
        start_node='ask',
        nodes=[
            Node(
                name='ask',
                content=content,
                options=(
                    lambda:
                    _get_last_n_list(
                        options(),
                        max_num_options,
                        new_entry_text_choice
                    ) + [new_entry_text_choice]
                ),
                message_type=Message.Type.MULTIPLE_CHOICE,
                result_db_key=save_db_key,
                is_append_result=is_append_result,
                text_populator=text_populator,
                transitions=['exit'] * max_num_options + ["user entry"]
            ),
            Node(
                name='user entry',
                content=new_entry_message,
                options=new_entry_options,
                message_type=Message.Type.DIRECT_INPUT,
                result_db_key=save_db_key,
                is_append_result=is_append_result,
                result_convert_from_str_fn=result_convert_from_str_fn,
                tests=tests,
                error_message=new_entry_error_message,
                text_populator=text_populator,
                transitions='exit',
            )
        ]
    )


def _get_last_n_list(list_, n, exclude_values=None):
    exclude_values = lists.make_sure_is_iterable(exclude_values)
    list_ = lists.remove_repeats(list_, is_remove_from_front=False)
    out = []
    idx = len(list_)-1
    while len(out) < n and idx >= 0:
        value = list_[idx]
        if value not in exclude_values:
            out.append(value)
        idx -= 1

    return out


