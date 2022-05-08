from interaction_engine.engine import InteractionEngine
from interfaces.cordial_interface import ERROR_RESPONSE


class VisionInteractionEngine(InteractionEngine):

    def modified_run(self, planner):
        while planner.is_active:
            for node_name in self.modified_run_next_plan(planner):
                yield node_name

    def modified_run_next_plan(self, planner):
        message_name, pre_hook, post_hook = planner.pop_plan(is_return_hooks=True)
        run_post_hook = True
        yield message_name

        messager = self._messagers[message_name]
        messager.reset()

        pre_hook()
        while messager.is_active:
            msg = messager.get_message()
            try:
                user_response = self._interface.run(msg)
                if user_response == ERROR_RESPONSE:
                    run_post_hook = False
                    break
                messager.transition(user_response)
            except Exception as e:
                print(e)
                if run_post_hook:
                    print("Error in interaction, running post-hook")
                    post_hook()

        if run_post_hook:
            post_hook()
