from math import atan
import unittest

import mavlink_all as mavlink
from utils.model_utils import STD_G
from trivial_model import Controls, simulate, MAX_ANGLE

# pyright: reportUntypedBaseClass=false
class TestSimulate(unittest.TestCase):
    def default_state(_):
        return mavlink.MAVLink_sim_state_message(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0 ,0 ,0, 0, 0, 0, 0)    

    def test_zero(self):
        previous_state = self.default_state()
        state = simulate(previous_state, Controls(), 0.1)

        self.assertEqual(state.xacc, 0.0)
        self.assertEqual(state.yacc, 0.0)
        self.assertEqual(state.zacc, -STD_G)


    def test_forward(self):
        """
        Lean the vehicle forward (nose down)
        """
        previous_state = self.default_state()
        controls = Controls()
        controls.pitch = 1.0
        state = simulate(previous_state, controls, 0.1)

        # controls is positive for stick forward, pitch is positive for nose up
        self.assertEqual(state.pitch, -MAX_ANGLE)

        # with nose down, the gravity is like accelerating up and back
        self.assertLess(state.xacc, 0)
        self.assertEqual(state.yacc, 0)
        self.assertLess(state.zacc, 0)

        # maximum tilt was used
        self.assertAlmostEqual(atan(state.xacc / state.zacc), MAX_ANGLE, 2)


    def test_right(self):
        """
        Lean the vehicle to the right
        """
        previous_state = self.default_state()
        controls = Controls()
        controls.roll = 1.0
        state = simulate(previous_state, controls, 0.1)

        # controls is positive for stick right, roll is positive to right
        self.assertEqual(state.roll, MAX_ANGLE)

        # with right wing down, the gravity is like accelerating up and left
        self.assertEqual(state.xacc, 0)
        self.assertLess(state.yacc, 0)
        self.assertLess(state.zacc, 0)

        # maximum tilt was used
        self.assertAlmostEqual(atan(state.yacc / state.zacc), MAX_ANGLE, 2)
