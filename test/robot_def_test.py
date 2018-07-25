import unittest
import sympy
from robot_def import RobotDef, new_sym


class RobotDefTestCase(unittest.TestCase):
    def setUp(self):
        q0, q1, q2, q3, q4, q5, q6, q7, q8, q9 = new_sym('q:10')
        self.rbt_def = RobotDef([(0,   -1, [1],    0, 0, 0, 0),
                      (1,   0,  [2],    0, 0, -0.21537, q1),
                      (2,   1,  [3],     0, -sympy.pi/2, 0, q2+sympy.pi/2)],
                     dh_convention='mdh',
                     friction_type=['Coulomb', 'viscous', 'offset'])


class RobotDefParamTestCase(RobotDefTestCase):
    def runTest(self):
        self.assertEquals(self.rbt_def.r,
                          [0, [new_sym('r1x'), new_sym('r1y'), new_sym('r1z')],
                           [new_sym('r2x'), new_sym('r2y'), new_sym('r2z')]],
                          "incorrect r symbolic definition")

        self.assertEquals(self.rbt_def.l,
                          [0, [new_sym('l1x'), new_sym('l1y'), new_sym('l1z')],
                           [new_sym('l2x'), new_sym('l2y'), new_sym('l2z')]],
                          "incorrect l symbolic definition")


class RobotDefCoordinateTestCase(RobotDefTestCase):
    def runTest(self):
        self.assertEquals(self.rbt_def.coordinates, [new_sym('q1'), new_sym('q2')], "incorrect coordinate definition")


def get_rbt_def_test_suite():
    suite = unittest.TestSuite()
    suite.addTest(RobotDefParamTestCase())
    suite.addTest(RobotDefCoordinateTestCase())
    return suite


rbt_def_test_suite = get_rbt_def_test_suite()

unittest.TextTestRunner(verbosity=2).run(rbt_def_test_suite)