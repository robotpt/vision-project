from interaction_engine.planner.base_planner import BasePlanner
from robotpt_common_utils import lists
from interaction_engine.messager.base_messenger import BaseMessenger


class MessagerPlanner(BasePlanner):

    def __init__(self, possible_plans):

        names = self._list_of_names(possible_plans)
        super().__init__(names)

    def insert(self, plan, pre_hook=None, post_hook=None):
        names = self._list_of_names(plan)
        super().insert(names, pre_hook, post_hook)

    def new_plan(self, plan, prehook=None, post_hook=None):
        names = self._list_of_names(plan)
        super().new_plan(names, prehook, post_hook)

    @staticmethod
    def _list_of_names(plans):
        plans = lists.make_sure_is_iterable(plans)
        for p in plans:
            if not issubclass(p.__class__, BaseMessenger):
                raise TypeError
        return [p.name for p in plans]
