import sympy
import cvxpy as cp
from pyOpt import Optimization
from pyOpt import pyNSGA2
from pyOpt import pySLSQP
import numpy as np



class TrajOptimizer:
    def __init__(self, order, base_freq, dyn, constraints):
        self._order = order
        self._base_freq = base_freq
        self._dyn = dyn
        self.constraints = constraints



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

        self.q = np.zeros((sample_num, self._dyn.rbt_def.dof))
        self.dq = np.zeros((sample_num, self._dyn.rbt_def.dof))
        self.ddq = np.zeros((sample_num, self._dyn.rbt_def.dof))

        self.H = np.zeros((self._dyn.rbt_def.dof * sample_num, self._dyn.rbt_def.base_num))

    def _obj(self, x):
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
        g = 0

        # fail
        fail = 0

        return f, g, fail

    def _add_vars_and_constraints(self):
        pass


    def optimize(self):

        pass