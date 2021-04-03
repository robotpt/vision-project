from interaction_engine.engine import InteractionEngine


class VisionInteractionEngine(InteractionEngine):

    def modified_run(self, planner):
        while planner.is_active:
            for node_name in self.modified_run_next_plan(planner):
                yield node_name

    def modified_run_next_plan(self, planner):
        message_name, pre_hook, post_hook = planner.pop_plan(is_return_hooks=True)
        yield message_name

        messager = self._messagers[message_name]
        messager.reset()

        pre_hook()
        while messager.is_active:
            msg = messager.get_message()
            user_response = self._interface.run(msg)
            messager.transition(user_response)
        post_hook()
