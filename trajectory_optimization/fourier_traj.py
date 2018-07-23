import numpy as np


class FourierTraj:
    def __init__(self, dof, order, base_freq, sample_num_per_period=10):
        self.dof = dof
        self.order = order
        self.base_freq = base_freq
        self.sample_num_per_period = sample_num_per_period
        self.sample_num = self.order * self.sample_num_per_period + 1

        self._gen_q_base()

    def _gen_q_base(self):
        period = 1.0 / self.base_freq
        self.t = np.linspace(0, period, num=self.sample_num)

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

                self.fourier_dq_base[n, o + 1] = np.sin(phase)
                self.fourier_dq_base[n, self.order + o + 1] = np.cos(phase)

                self.fourier_ddq_base[n, o + 1] = -c * np.sin(phase)
                self.fourier_ddq_base[n, self.order + o + 1] = c * np.cos(phase)

        print('fourier_q_base:')
        print(self.fourier_q_base)
        print('fourier_dq_base:')
        print(self.fourier_dq_base)
        print('fourier_ddq_base:')
        print(self.fourier_ddq_base)

    def fourier_base_x2q(self, x):
        q = np.zeros((self.sample_num, self.dof))
        dq = np.zeros((self.sample_num, self.dof))
        ddq = np.zeros((self.sample_num, self.dof))

        for d in range(self.dof):
            start = d * (2 * self.order + 1)
            end = (d + 1) * (2 * self.order + 1)
            q[:, d] = np.matmul(self.fourier_q_base, x[start:end])
            dq[:, d] = np.matmul(self.fourier_dq_base, x[start:end])
            ddq[:, d] = np.matmul(self.fourier_ddq_base, x[start:end])

            print('q{}: {}'.format(d, q[:, d]))
            print('dq{}: {}'.format(d, dq[:, d]))
            print('ddq{}: {}'.format(d, ddq[:, d]))

        return q, dq, ddq
