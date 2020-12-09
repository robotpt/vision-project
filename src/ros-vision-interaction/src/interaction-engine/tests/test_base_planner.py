import unittest
from interaction_engine.planner.base_planner import BasePlanner


class TestBasePlanner(unittest.TestCase):

    def test_order_of_input_plan(self):
        possible_plans = ['plan1', 'plan2', 'plan3', 1, 2, 3]
        s = BasePlanner(possible_plans)
        s.new_plan(possible_plans)
        for p in possible_plans:
            self.assertEqual(
                p,
                s.pop_plan()
            )

    def test_insert_into_plan(self):
        possible_plans = ['plan1', 'plan2', 'plan3', 1, 2, 3]
        s = BasePlanner(possible_plans)
        for p in possible_plans:
            s.insert(p)
        for p in possible_plans:
            self.assertEqual(
                p,
                s.pop_plan()
            )
        s.insert(possible_plans)
        for p in possible_plans:
            self.assertEqual(
                p,
                s.pop_plan()
            )

    def test_invalid_plans(self):
        possible_plans = 'the only true plan'
        s = BasePlanner(possible_plans)
        invalid_plans = [1, 2, 3, 't', 'h', 'e', None, 'foobar', int, input]
        for p in invalid_plans:
            self.assertRaises(
                ValueError,
                s.insert,
                p
            )
            self.assertRaises(
                ValueError,
                s.new_plan,
                p
            )

    def test_for_equal_plans(self):

        possible_plans = ['plan1', 'plan2', 'plan3', 1, 2, 3]
        p1 = BasePlanner(possible_plans)
        p2 = BasePlanner(possible_plans)
        p3 = BasePlanner(possible_plans)
        self.assertEqual(p1, p2)
        self.assertEqual(p1, p3)

        p1.insert(possible_plans)
        p2.insert(possible_plans)
        p3.insert(possible_plans[:-3])
        self.assertEqual(p1, p2)
        self.assertNotEqual(p1, p3)

    def test_pre_hooks(self):

        var = 0

        def set_var_fn(value):
            nonlocal var

            def fn():
                nonlocal var
                var = value
            return fn

        possible_plans = ['plan1', 'plan2', 'plan3', 1, 2, 3]
        planner = BasePlanner(possible_plans)

        for i in range(1, 4):
            # no args means identity functions are attached
            planner.insert(i)

            # these functions will modify 'var'
            planner.insert(
                i,
                pre_hook=set_var_fn(i),
                post_hook=set_var_fn(2*i)
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
            self.assertEqual(2*i, var)
