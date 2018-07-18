import sympy
import numpy as np
from collections import deque
from utils import vec2so3


class Dynamics:
    def __init__(self, kin, g=[0, 0, -9.81]):
        self._kin = kin
        self._g = np.matrix(g)

    def _ml2r(self, m, l):
        return sympy.Matrix(l / m)

    def _Lmr2I(self, L, m, r):
        return sympy.Matrix(L - m * vec2so3(r).transpose() * vec2so3(r))

    def cal_dynamics(self):
        print("Calculating dynamics")
        p_e = 0
        k_e = 0

        node_q = deque([self._kin._dh_root])
        while len(node_q) != 0:
            node = node_q.pop()
            print("Frame number: " + str(node._num))
            if node.get_prev_link() is not None:
                r = self._ml2r(node._m, node._l)
                k_e += sympy.simplify(node._m * node.v_cw.dot(node.v_cw) / 2 +
                                      node.w_cb.transpose() * self._Lmr2I(node._L_mat, node._m, r) / 2)
                p_e += -node._m * node.pos_c.dot(self._g)

            for s in node.get_succ_link():
                node_q.append(s)

        lagrangian = k_e - p_e

        print(lagrangian)


