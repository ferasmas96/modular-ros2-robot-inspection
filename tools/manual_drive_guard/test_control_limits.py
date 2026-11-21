import math
import random
import unittest
from control_limits import ManualGate, ZERO, bounded


class LimitsTest(unittest.TestCase):
    def test_previous_excessive_keys_are_hard_clamped(self):
        self.assertEqual(bounded((1, 0, 0, 0, 0, .5)), (.1, 0, 0, 0, 0, .1))
        self.assertEqual(bounded((-1, 0, 0, 0, 0, -.5)), (-.1, 0, 0, 0, 0, -.1))

    def test_other_axes_are_zero(self):
        self.assertEqual(bounded((.05, 3, 4, 5, 6, -.05)), (.05, 0, 0, 0, 0, -.05))

    def test_nonfinite_never_produces_a_command(self):
        for value in [math.nan, math.inf, -math.inf]:
            g = ManualGate(); g.receive((value, 0, 0, 0, 0, 0), 0)
            self.assertIsNotNone(g.fault)
            self.assertEqual(g.sample(0), (ZERO, ZERO))

    def test_disarmed_preview_is_bounded_robot_output_zero(self):
        g = ManualGate(); g.clock(10**9, 0); g.receive((1, 0, 0, 0, 0, .5), .1)
        self.assertEqual(g.sample(.2), ((.1, 0, 0, 0, 0, .1), ZERO))

    def test_arm_requires_released_input_and_fresh_post_arm_event(self):
        g = ManualGate(); g.clock(10**9, 0); g.receive((1, 0, 0, 0, 0, 0), .1)
        self.assertFalse(g.arm(.2))
        g.receive(ZERO, .2); self.assertTrue(g.arm(.2))
        self.assertEqual(g.sample(.3), (ZERO, ZERO))
        g.receive((1, 0, 0, 0, 0, 0), .3)
        self.assertEqual(g.sample(.31)[1][0], .1)

    def test_stale_input_stops_even_with_advancing_simulation(self):
        g = ManualGate(); g.clock(1, 0); self.assertTrue(g.arm(0))
        g.receive((1, 0, 0, 0, 0, 0), 0); g.clock(2, .4)
        self.assertEqual(g.sample(.4), (ZERO, ZERO))

    def test_clock_stall_latches_stop(self):
        g = ManualGate(); g.clock(1, 0); g.arm(0); g.receive((1, 0, 0, 0, 0, 0), .4)
        self.assertEqual(g.sample(.6), (ZERO, ZERO)); self.assertFalse(g.armed)
        g.clock(2, .61); self.assertFalse(g.arm(.61))

    def test_clock_rewind_and_arm_lease_fail_closed(self):
        g = ManualGate(); g.clock(10, 0); g.arm(0); g.clock(9, .1)
        self.assertFalse(g.armed); self.assertIsNotNone(g.fault)
        g = ManualGate(); g.clock(1, 0); g.arm(0); g.clock(2, 120)
        self.assertEqual(g.sample(120), (ZERO, ZERO)); self.assertFalse(g.armed)

    def test_random_finite_inputs_are_always_bounded(self):
        rng = random.Random(4004)
        for _ in range(1000):
            v = bounded(tuple(rng.uniform(-100, 100) for _ in range(6)))
            self.assertLessEqual(abs(v[0]), .1); self.assertLessEqual(abs(v[5]), .1)
            self.assertEqual(v[1:5], (0, 0, 0, 0))

    def test_repeated_arm_cannot_extend_authorized_window(self):
        g = ManualGate(); g.clock(1, 0); self.assertTrue(g.arm(0))
        g.clock(2, 100); self.assertFalse(g.arm(100))
        self.assertEqual(g.armed_wall, 0)


if __name__ == '__main__':
    unittest.main()
