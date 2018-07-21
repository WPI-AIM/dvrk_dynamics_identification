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
    def __init__(self, rbt_def, geom, g=[0, 0, -9.81]):
        self._rbt_def = rbt_def
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

        for num in self._rbt_def.link_nums[1:]:
            p_e += -self._rbt_def.m[num] * self._geom.p_c[num].dot(self._g)

            k_e_n = self._rbt_def.m[num] * self._geom.v_cw[num].dot(self._geom.v_cw[num])/2 +\
                   (self._geom.w_b[num].transpose() * self._rbt_def.I_by_Llm[num] * self._geom.w_b[num])[0, 0]/2
            k_e_n = sympy.simplify(k_e_n)
            print('k_e:', k_e_n)
            k_e += k_e_n

        # Lagrangian
        L = k_e - p_e

        tau = []
        print(len(self._rbt_def.coordinates))
        for q, dq in zip(self._rbt_def.coordinates, self._rbt_def.d_coordinates):
            dk_ddq = sympy.diff(k_e, dq)
            dk_ddq_t = dk_ddq.subs(self._rbt_def.subs_q2qt + self._rbt_def.subs_dq2dqt)
            dk_ddq_dtt = sympy.diff(dk_ddq_t, sympy.Symbol('t'))
            print('dk_ddq_dtt:')
            print(dk_ddq_dtt)
            dk_ddq_dt = dk_ddq_dtt.subs(self._rbt_def.subs_ddqt2ddq + self._rbt_def.subs_dqt2dq + self._rbt_def.subs_qt2q)
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
        A, b = sympy.linear_eq_to_matrix(self.tau, self._rbt_def.params)
        print('A:')
        print(A)
        print(A.shape)
        self.H = A
        print('b:')
        print(b)
        print('Ax - b:')
        print(sympy.simplify(A*sympy.Matrix(self._rbt_def.params) - sympy.Matrix(self.tau)))

        input_vars = tuple(self._rbt_def.coordinates + self._rbt_def.d_coordinates + self._rbt_def.dd_coordinates)
        print('input_vars', input_vars)
        self.H_func = sympy.lambdify(input_vars, self.H)
        print(self.H_func)
        start_time = time.time()
        print(self.H_func(*np.random.random_sample((len(input_vars),))))
        print('time: ', time.time() - start_time)

    def _calc_base_param(self):
        r, P_X, P = find_dyn_parm_deps(len(self._rbt_def.coordinates), len(self._rbt_def.params), self.H_func)
        self.base_num = r
        print('base number: ', self.base_num)
        self.base_param = P_X.dot(np.matrix(self._rbt_def.params).transpose())
        print('base parameters: ', self.base_param)
        print(P)
        P_b = P[:r].tolist()
        print(type(P_b))
        print(self.H)

        self.H_b = self.H[:, P_b]
        print('H_b: ', self.H_b)

        print('error: ', sympy.simplify(self.H * np.matrix(self._rbt_def.params).transpose() - self.H_b * self.base_param))

    def _calc_M(self):
        A, b = sympy.linear_eq_to_matrix(self.tau, self._rbt_def.dd_coordinates)
        print('dd:', self._rbt_def.dd_coordinates)
        print('tau:')
        print(A[0, :])
        print(A[1, :])
        self.M = A
        print('M:')
        print(self.M.shape)
        print(A)


    def _calc_G(self):
        subs_qdq2zero = [(dq, 0) for dq in self._rbt_def.d_coordinates]
        subs_qdq2zero += [(ddq, 0) for ddq in self._rbt_def.dd_coordinates]
        self.G = sympy.Matrix(self.tau).subs(subs_qdq2zero)
        print('G:')
        print(self.G)

    def _calc_C(self):
        subs_ddq2zero = [(ddq, 0) for ddq in self._rbt_def.dd_coordinates]
        self.C = sympy.Matrix(self.tau).subs(subs_ddq2zero) - self.G
        print('C:')
        print(self.C)

    def _calc_MCG(self):
        self._calc_M()
        self._calc_G()
        self._calc_C()


class Dynamics2:
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



