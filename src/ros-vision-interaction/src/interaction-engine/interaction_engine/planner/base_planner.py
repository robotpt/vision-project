from robotpt_common_utils import lists


def empty_fn():
    pass


class BasePlanner:

    def __init__(self, possible_plans):
        possible_plans = lists.make_sure_is_iterable(possible_plans)
        self._possible_items = possible_plans.copy()

        self._plan = []
        self._pre_hook = []
        self._post_hook = []

    def insert(self, plan, pre_hook=None, post_hook=None):
        plan = lists.make_sure_is_iterable(plan)
        if not self._is_valid_plan(plan):
            raise ValueError(f"Invalid plan: '{plan}'")

        if pre_hook is not None:
            pre_hook = lists.make_sure_is_iterable(pre_hook)
        else:
            pre_hook = [None] * len(plan)
        if len(plan) != len(pre_hook):
            raise ValueError("Pre hooks must be the same length as plans")

        if post_hook is not None:
            post_hook = lists.make_sure_is_iterable(post_hook)
        else:
            post_hook = [None] * len(plan)
        if len(plan) != len(post_hook):
            raise ValueError("Post hooks must be the same length as plans")

        for i in range(len(plan)):
            self._insert_one(plan[i], pre_hook[i], post_hook[i])

    def _insert_one(self, plan, pre_hook, post_hook):

        if pre_hook is not None:
            if not callable(pre_hook):
                raise ValueError("Pre hook should be callable")
        else:
            pre_hook = empty_fn

        if post_hook is not None:
            if not callable(pre_hook):
                raise ValueError("Post hook should be callable")
        else:
            post_hook = empty_fn

        self._plan.append(plan)
        self._pre_hook.append(pre_hook)
        self._post_hook.append(post_hook)

    def update_last_inserts_hooks(self, pre_hook=None, post_hook=None):
        _plan = self._plan.pop()
        _pre_hook = self._pre_hook.pop()
        _post_hook = self._post_hook.pop()
        if pre_hook is not None:
            _pre_hook = pre_hook
        if post_hook is not None:
            _post_hook = post_hook
        self._insert_one(_plan, _pre_hook, _post_hook)

    def new_plan(self, plan, pre_hooks=None, post_hook=None):
        plan = lists.make_sure_is_iterable(plan)
        if not self._is_valid_plan(plan):
            raise ValueError("Invalid plan")
        self._plan = []
        self.insert(plan, pre_hooks, post_hook)

    def _is_valid_plan(self, plan):
        plan = lists.make_sure_is_iterable(plan)
        for p in plan:
            if p not in self._possible_items:
                return False
        return True

    @property
    def plan(self):
        return self._plan

    def pop_plan(self, is_return_hooks=False):
        if is_return_hooks:
            return self._plan.pop(0), self._pre_hook.pop(0), self._post_hook.pop(0)
        else:
            return self._plan.pop(0)

    def clear_all(self):
        self._plan = []

    @property
    def is_active(self):
        return len(self._plan) > 0

    def __eq__(self, other):
        if not issubclass(other.__class__, BasePlanner):
            raise ValueError
        if len(self.plan) != len(other.plan):
            return False
        for i in range(len(self.plan)):
            if self.plan[i] != other.plan[i]:
                return False
        return True

    def __repr__(self):
        return str(self.plan)



