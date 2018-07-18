import sympy
import numpy as np
from collections import deque
from utils import vec2so3
from dh_def import new_sym
from sympy import pprint


class Dynamics:
    def __init__(self, kin, g=[0, 0, -9.81]):
        self._kin = kin
        self._g = np.matrix(g)
        coordinates = self._kin.get_coordinates()
        self._tau = []

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
                k_e += node._m*node.v_cw.dot(node.v_cw)/2 +\
                                      (node.w_b.transpose()*self._Lmr2I(node._L_mat, node._m, r)*node.w_b)[0, 0]/2
                p_e += -node._m * node.pos_c.dot(self._g)

                print(k_e)
                print(p_e)

            for s in node.get_succ_link():
                node_q.append(s)

        lagrangian = k_e - p_e

        print('lagrangian:')
        print(lagrangian)
        diff_dq = [sympy.diff(k_e, dq) for dq in self._kin._d_coordinates]
        print('diff_dq:')
        print(diff_dq)
        diff_dq_t = [diff_dq_i.subs(self._kin._subs_dq2dqt + self._kin._subs_q2qt) for diff_dq_i in diff_dq]
        print('diff_dq_t: ')
        print(diff_dq_t)
        diff_dq_dtt = [sympy.diff(diff_dq_t_i, sympy.Symbol('t')) for diff_dq_t_i in diff_dq_t]
        print('diff_dq_dtt:')
        print(diff_dq_dtt)
        diff_dq_dt = [e.subs(self._kin._subs_ddqt2ddq + self._kin._subs_dqt2dq + self._kin._subs_qt2q) for e in diff_dq_dtt]
        print('diff_dq_dt:')
        print(diff_dq_dt)

        dL_dq = [sympy.diff(lagrangian, q) for q in self._kin.get_coordinates()]
        print('dL_dq:')
        print(dL_dq)

        self._tau = [a - b for a, b in zip(diff_dq_dt, dL_dq)]
        print('tau:')
        print(self._tau)
        m_eq = sympy.Matrix(self._tau).vec()
        print('m_eq')
        print(m_eq)
        A, b = sympy.linear_eq_to_matrix(self._tau, self._kin._dd_coordinates)
        print('matrix A:')
        sympy.pprint(A)
        print('b:')
        print(b)

        lagrangian_t = lagrangian.subs(self._kin._subs_q2qt + self._kin._subs_dq2dqt)
        print('lagrangian_t:')
        print(lagrangian_t)



