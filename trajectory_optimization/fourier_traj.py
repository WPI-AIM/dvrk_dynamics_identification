import numpy as np
import math

verbose = False

if verbose:
    def vprint(*args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        for arg in args:
           print arg,
        print
else:
    vprint = lambda *a: None      # do-nothing function


class FourierTraj:
    def __init__(self, dof, order, base_freq, sample_num_per_period=10, frequency='nan', final_time=10):
        self.dof = dof
        self.order = order
        self.base_freq = base_freq
        self.sample_num_per_period = sample_num_per_period

        # if no specified frequency and final_time, generate a one-period trajectory.
        if math.isnan(float(frequency)):
            self.sample_num = self.order * self.sample_num_per_period + 1
            self.period = 1.0 / self.base_freq
        else:
            self.sample_num = frequency * final_time
            self.period = final_time


        self.q = np.zeros((self.sample_num, self.dof))
        self.dq = np.zeros((self.sample_num, self.dof))
        self.ddq = np.zeros((self.sample_num, self.dof))

        self._gen_q_base()

    def _gen_q_base(self):
        self.t = np.linspace(0, self.period, num=self.sample_num)

        self.fourier_q_base = np.zeros((self.sample_num, 2 * self.order + 1))
        self.fourier_dq_base = np.zeros((self.sample_num, 2 * self.order + 1))
        self.fourier_ddq_base = np.zeros((self.sample_num, 2 * self.order + 1))

        for n in range(self.sample_num):
            self.fourier_q_base[n, 0] = 1
            for o in range(self.order):
                phase = 2 * np.pi * (o + 1) * self.t[n] * self.base_freq

                c = 2 * np.pi * (o + 1) * self.base_freq
                self.fourier_q_base[n, o + 1] = np.sin(phase) / c
                self.fourier_q_base[n, self.order + o + 1] = -np.cos(phase) / c

                self.fourier_dq_base[n, o + 1] = np.cos(phase)
                self.fourier_dq_base[n, self.order + o + 1] = np.sin(phase)

                self.fourier_ddq_base[n, o + 1] = -c * np.sin(phase)
                self.fourier_ddq_base[n, self.order + o + 1] = c * np.cos(phase)

        vprint('fourier_q_base:')
        vprint(self.fourier_q_base)
        vprint('fourier_dq_base:')
        vprint(self.fourier_dq_base)
        vprint('fourier_ddq_base:')
        vprint(self.fourier_ddq_base)

    def fourier_base_x2q(self, x):
        for d in range(self.dof):
            start = d * (2 * self.order + 1)
            end = (d + 1) * (2 * self.order + 1)
            self.q[:, d] = np.matmul(self.fourier_q_base, x[start:end])
            self.dq[:, d] = np.matmul(self.fourier_dq_base, x[start:end])
            self.ddq[:, d] = np.matmul(self.fourier_ddq_base, x[start:end])
        return self.q, self.dq, self.ddq
