import sympy
import cvxpy as cp
from pyOpt import Optimization
from pyOpt import pyNSGA2
from pyOpt import pySLSQP
import pyOpt
import numpy as np


q0_scale = np.pi
fourier_scale = 1

# joint constraints
# [(joint_var, q_low, q_upper, dq_low, dq_upper), ..., (...)]

# cartesian constraints
# [(joint_num, x_low, x_high, y_low, y_high, z_low, z_high), ..., (...)]

class TrajOptimizer:
    def __init__(self, dyn, order, base_freq, joint_constraints=[], cartesian_constraints = [],
                 q0_min=-q0_scale, q0_max=q0_scale,
                 ab_min=-fourier_scale, ab_max=fourier_scale):
        self._order = order
        self._base_freq = base_freq
        self._dyn = dyn
        self._joint_constraints = joint_constraints
        self._joint_const_num = len(self._joint_constraints)
        print('joint constraint number: ', self._joint_const_num)
        self._cartesian_constraints = cartesian_constraints
        self._cartesian_const_num = len(self._cartesian_constraints)
        print('cartisian constraint number: ', self._cartesian_const_num)
        self._const_num = self._joint_const_num * 4 + self._cartesian_const_num * 6
        print('constraint number: ', self._const_num)

        self._q0_min = q0_min
        self._q0_max = q0_max
        self._ab_min = ab_min
        self._ab_max = ab_max

        self._prepare_opt()

    def _prepare_opt(self):
        sample_num = self._order * 10 + 1
        self.sample_num = sample_num

        period = 1.0/self._base_freq
        t = np.linspace(0, period, num=sample_num)

        self.fourier_q_base = np.zeros((sample_num, 2*self._order + 1))
        self.fourier_dq_base = np.zeros((sample_num, 2 * self._order + 1))
        self.fourier_ddq_base = np.zeros((sample_num, 2 * self._order + 1))

        for n in range(self.sample_num):
            self.fourier_q_base[n, 0] = 1
            for o in range(self._order):
                phase = 2 * np.pi * (o + 1) * t[n] * self._base_freq

                c = 2 * np.pi * (o + 1) * self._base_freq
                self.fourier_q_base[n, o+1] = np.sin(phase) / c
                self.fourier_q_base[n, self._order+o+1] = -np.cos(phase) / c

                self.fourier_dq_base[n, o+1] = np.sin(phase)
                self.fourier_dq_base[n, self._order+o+1] = np.cos(phase)

                self.fourier_ddq_base[n, o+1] = -c * np.sin(phase)
                self.fourier_ddq_base[n, self._order+o+1] = c * np.cos(phase)

        print('fourier_q_base:')
        print(self.fourier_q_base)
        print('fourier_dq_base:')
        print(self.fourier_dq_base)
        print('fourier_ddq_base:')
        print(self.fourier_ddq_base)


        self.q = np.zeros((sample_num, self._dyn.rbt_def.dof))
        self.dq = np.zeros((sample_num, self._dyn.rbt_def.dof))
        self.ddq = np.zeros((sample_num, self._dyn.rbt_def.dof))

        self.H = np.zeros((self._dyn.rbt_def.dof * sample_num, self._dyn.base_num))


    def _obj_func(self, x):
        # objective
        for d in range(self._dyn.rbt_def.dof):
            start = d * (2 * self._order + 1)
            end = (d + 1) * (2 * self._order + 1)
            self.q[:, d] = np.matmul(self.fourier_q_base, x[start:end])
            self.dq[:, d] = np.matmul(self.fourier_dq_base, x[start:end])
            self.ddq[:, d] = np.matmul(self.fourier_ddq_base, x[start:end])

        for n in range(self.sample_num):
            vars_input = self.q[n, :].tolist() + self.dq[n, :].tolist() + self.ddq[n, :].tolist()
            self._dyn.H_b_func(*vars_input)

        f = np.linalg.cond(self.H)

        # constraint
        g = [0.0]*self._const_num
        g_cnt = 0

        # Joint constraints
        for j_c in self._joint_constraints:
            q, q_l, q_u, dq_l, dq_u = j_c
            co_num = self._dyn.rbt_def.coordinates.index(q)
            for qt, dqt in zip(self.q[:, co_num], self.dq[:, co_num]):
                g[g_cnt] = qt - q_u
                g_cnt += 1
                g[g_cnt] = q_l - qt
                g_cnt += 1
                g[g_cnt] = dqt - dq_u
                g_cnt += 1
                g[g_cnt] = dq_l - dqt
                g_cnt += 1
        print('constraints number: ', g_cnt)
        # Cartesian Constraints

        # fail
        fail = 0

        return f, g, fail

    def _add_obj2prob(self):
        self._opt_prob.addObj('f')

    def _add_vars2prob(self):
        joint_coef_num = 2*self._order + 1

        def rand_local(l, u, scale):
            return (np.random.random() * (u - l)/2 + (u + l)/2) * scale

        for num in range(self._dyn.rbt_def.dof):
            # q0
            self._opt_prob.addVar('x'+str(num*joint_coef_num + 1), 'c',
                                  lower=self._q0_min, upper=self._q0_max,
                                  value=rand_local(self._q0_min, self._q0_max, 0.1))
            for o in range(self._order):
                # a sin
                self._opt_prob.addVar('x' + str(num * joint_coef_num + 1 + o + 1), 'c',
                                      lower=self._ab_min, upper=self._ab_max,
                                      value=rand_local(self._ab_min, self._ab_max, 0.1))
                # b cos
                self._opt_prob.addVar('x' + str(num * joint_coef_num + 1 + self._order + o + 1), 'c',
                                      lower=self._ab_min, upper=self._ab_max,
                                      value=rand_local(self._ab_min, self._ab_max, 0.1))

    def _add_const2prob(self):
        self._opt_prob.addConGroup('g', self._const_num * self.sample_num, type='i', lower=0.0, upper=np.inf)

        # for c_c in self._cartesian_constraints:
        #     self._dyn.p_n[]
        # pass

    def optimize(self):
        self._opt_prob = pyOpt.Optimization('Optimial Excitation Trajectory', self._obj_func)
        self._add_vars2prob()
        self._add_obj2prob()
        self._add_const2prob()

        print(self._opt_prob)


        pass