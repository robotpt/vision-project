from interaction_engine.planner.messanger_planner import MessagerPlanner
from interaction_engine.messager import Message

import unittest

m = Message(
    content='name',
    options='foo',
    message_type=Message.Type.MULTIPLE_CHOICE,
)

class TestMessagePlanner(unittest.TestCase):

    def test_get_names(self):

        possible_plans = [m]
        message_planner = MessagerPlanner(possible_plans)
        self.assertEqual(
            [],
            message_planner.plan
        )
        message_planner.insert(m)
        message_planner.insert(m)
        message_planner.insert(m)
        self.assertEqual(
            ['name']*3,
            message_planner.plan
        )

    def test_hooks(self):

        var = 0

        def set_var_fn(value):
            nonlocal var

            def fn():
                nonlocal var
                var = value

            return fn

        possible_plans = [m]
        planner = MessagerPlanner(possible_plans)

        for i in range(1, 4):
            # no args means identity functions are attached
            planner.insert(m)

            # these functions will modify 'var'
            planner.insert(
                m,
                pre_hook=set_var_fn(i),
                post_hook=set_var_fn(2 * i)
            )

        self.assertEqual(0, var)
        for i in range(1, 4):
            var_at_beginning_of_loop = var

            # check nothing modified with identity functions
            plan, pre_hook, post_hook = planner.pop_plan(is_return_hooks=True)
            pre_hook()
            self.assertEqual(var_at_beginning_of_loop, var)
            post_hook()
            self.assertEqual(var_at_beginning_of_loop, var)

            # check var is modified by functions
            plan, pre_hook, post_hook = planner.pop_plan(is_return_hooks=True)
            pre_hook()
            self.assertEqual(i, var)
            post_hook()
            self.assertEqual(2 * i, var)
