import sympy
from sympy import lambdify
from sympy.utilities.iterables import flatten
import numpy as np
from collections import deque
from utils import vec2so3
from dyn_param_dep import find_dyn_parm_deps
from dh_def import new_sym
from sympy import pprint
import time


class Dynamics:
    def __init__(self, rbt_def, geom, g=[0, 0, -9.81], verbose=False):
        self.rbt_def = rbt_def
        self._geom = geom
        self._g = np.matrix(g)

        self._calc_dyn()
        self._calc_regressor()
        self._calc_MCG()
        self._calc_base_param()

    def _ml2r(self, m, l):
        return sympy.Matrix(l) / m

    def _Lmr2I(self, L, m, r):
        return sympy.Matrix(L - m * vec2so3(r).transpose() * vec2so3(r))

    def _calc_dyn(self):
        # Calculate kinetic energy and potential energy
        p_e = 0
        k_e = 0

        for num in self.rbt_def.link_nums[1:]:
            p_e += -self.rbt_def.m[num] * self._geom.p_c[num].dot(self._g)

            k_e_n = self.rbt_def.m[num] * self._geom.v_cw[num].dot(self._geom.v_cw[num])/2 +\
                   (self._geom.w_b[num].transpose() * self.rbt_def.I_by_Llm[num] * self._geom.w_b[num])[0, 0]/2
            k_e_n = sympy.simplify(k_e_n)
            print('k_e:', k_e_n)
            k_e += k_e_n

        # Lagrangian
        L = k_e - p_e

        tau = []
        print(len(self.rbt_def.coordinates))
        for q, dq in zip(self.rbt_def.coordinates, self.rbt_def.d_coordinates):
            dk_ddq = sympy.diff(k_e, dq)
            dk_ddq_t = dk_ddq.subs(self.rbt_def.subs_q2qt + self.rbt_def.subs_dq2dqt)
            dk_ddq_dtt = sympy.diff(dk_ddq_t, sympy.Symbol('t'))
            print('dk_ddq_dtt:')
            print(dk_ddq_dtt)
            dk_ddq_dt = dk_ddq_dtt.subs(self.rbt_def.subs_ddqt2ddq + self.rbt_def.subs_dqt2dq + self.rbt_def.subs_qt2q)
            print('dk_ddq_dt:')
            print(dk_ddq_dt)

            dL_dq = sympy.diff(L, q)
            print('dL_dq:')
            print(dL_dq)

            tau.append(sympy.simplify(dk_ddq_dt - dL_dq))
        print('tau: ')
        print(tau)
        self.tau = tau

    def _calc_regressor(self):
        A, b = sympy.linear_eq_to_matrix(self.tau, self.rbt_def.params)
        print('A:')
        print(A)
        print(A.shape)
        self.H = A
        print('b:')
        print(b)
        print('Ax - b:')
        print(sympy.simplify(A*sympy.Matrix(self.rbt_def.params) - sympy.Matrix(self.tau)))

        input_vars = tuple(self.rbt_def.coordinates + self.rbt_def.d_coordinates + self.rbt_def.dd_coordinates)
        print('input_vars', input_vars)
        self.H_func = sympy.lambdify(input_vars, self.H)
        print(self.H_func)
        start_time = time.time()
        print(self.H_func(*np.random.random_sample((len(input_vars),))))
        print('time: ', time.time() - start_time)

    def _calc_base_param(self):
        r, P_X, P = find_dyn_parm_deps(len(self.rbt_def.coordinates), len(self.rbt_def.params), self.H_func)
        self.base_num = r
        print('base number: ', self.base_num)
        self.base_param = P_X.dot(np.matrix(self.rbt_def.params).transpose())
        print('base parameters: ', self.base_param)
        print(P)
        P_b = P[:r].tolist()
        print(type(P_b))
        print(self.H)

        self.H_b = self.H[:, P_b]
        print('H_b: ', self.H_b)

        print('error: ', sympy.simplify(self.H * np.matrix(self.rbt_def.params).transpose() - self.H_b * self.base_param))

        input_vars = tuple(self.rbt_def.coordinates + self.rbt_def.d_coordinates + self.rbt_def.dd_coordinates)
        print('input_vars', input_vars)
        self.H_b_func = sympy.lambdify(input_vars, self.H_b)

    def _calc_M(self):
        A, b = sympy.linear_eq_to_matrix(self.tau, self.rbt_def.dd_coordinates)
        print('dd:', self.rbt_def.dd_coordinates)
        print('tau:')
        print(A[0, :])
        print(A[1, :])
        self.M = A
        print('M:')
        print(self.M.shape)
        print(A)


    def _calc_G(self):
        subs_qdq2zero = [(dq, 0) for dq in self.rbt_def.d_coordinates]
        subs_qdq2zero += [(ddq, 0) for ddq in self.rbt_def.dd_coordinates]
        self.G = sympy.Matrix(self.tau).subs(subs_qdq2zero)
        print('G:')
        print(self.G)

    def _calc_C(self):
        subs_ddq2zero = [(ddq, 0) for ddq in self.rbt_def.dd_coordinates]
        self.C = sympy.Matrix(self.tau).subs(subs_ddq2zero) - self.G
        print('C:')
        print(self.C)

    def _calc_MCG(self):
        self._calc_M()
        self._calc_G()
        self._calc_C()
