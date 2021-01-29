from interaction_engine.engine import InteractionEngine


class VisionInteractionEngine(InteractionEngine):

    def run(self):
        while self._plan.is_active:
            for node_name in self.run_next_plan():
                yield node_name

    def run_next_plan(self):
        message_name, pre_hook, post_hook = self._plan.pop_plan(is_return_hooks=True)
        yield message_name

        messager = self._messagers[message_name]
        messager.reset()

        pre_hook()
        while messager.is_active:
            msg = messager.get_message()
            user_response = self._interface.run(msg)
            messager.transition(user_response)
        post_hook()
