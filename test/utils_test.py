import unittest
import sympy
from utils import *
import numpy as np


class UtilsTestCase(unittest.TestCase):
    def test_gen_DLki_mat(self):
        DLkis = gen_DLki_mat()
        #params = list(new_sym("Lxx, Lxy, Lxz, Lyy, Lyz, Lzz, lx, ly, lz, m"))
        params = np.linspace(1, 10)
        D = np.zeros((6, 6))
        for i in range(10):
            D += sympy.Matrix(DLkis[i]) * params[i]
        sympy.pprint(D)

        generalized_inertia = sympy.Matrix([[0, params[1],   params[2],  0,            params[8],  -params[7]],
                                           [0, 0,           params[4],  -params[8],   0,          params[6]],
                                           [0, 0,           0,          params[7],    -params[6], 0],
                                           [0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0]])
        generalized_inertia = generalized_inertia.transpose() + generalized_inertia
        generalized_inertia[0, 0] = params[0]
        generalized_inertia[1, 1] = params[3]
        generalized_inertia[2, 2] = params[5]
        generalized_inertia[3, 3] = params[9]
        generalized_inertia[4, 4] = params[9]
        generalized_inertia[5, 5] = params[9]

        sympy.pprint(generalized_inertia)

        #self.assertEquals(D, generalized_inertia)
        self.assertAlmostEquals(D, generalized_inertia)



if __name__ == '__main__':
    sympy.init_printing()
    unittest.main()