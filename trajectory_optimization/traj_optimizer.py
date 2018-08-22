from pyOpt import pySLSQP
import pyOpt
import numpy as np
import matplotlib.pyplot as plt
from fourier_traj import FourierTraj

import sympy


q0_scale = 4*np.pi
fourier_scale = np.pi

# joint constraints
# [(joint_var, q_low, q_upper, dq_low, dq_upper), ..., (...)]

# cartesian constraints
# [(joint_num, x_low, x_high, y_low, y_high, z_low, z_high), ..., (...)]

class TrajOptimizer:
    def __init__(self, dyn, order, base_freq, joint_constraints=[], cartesian_constraints=[],
                 q0_min=-q0_scale, q0_max=q0_scale,
                 ab_min=-fourier_scale, ab_max=fourier_scale, verbose=False):
        self._order = order
        self._base_freq = base_freq
        self._dyn = dyn
        self._joint_constraints = joint_constraints
        self._joint_const_num = len(self._joint_constraints)
        print('joint constraint number: {}'.format(self._joint_const_num))
        self._cartesian_constraints = cartesian_constraints
        self._cartesian_const_num = len(self._cartesian_constraints)
        print('cartesian constraint number: {}'.format(self._cartesian_const_num))
        self._const_num = self._joint_const_num * 4 + self._cartesian_const_num * 6
        print('constraint number: {}'.format(self._const_num))

        self._q0_min = q0_min
        self._q0_max = q0_max
        self._ab_min = ab_min
        self._ab_max = ab_max

        # sample number for the highest term
        self._sample_point = 12

        self.fourier_traj = FourierTraj(self._dyn.dof, self._order, self._base_freq,
                                        sample_num_per_period=self._sample_point)

        self._prepare_opt()

        self.frame_pos = np.zeros((self.sample_num, 3))
        self.const_frame_ind = np.array([])

        for c_c in self._cartesian_constraints:

            frame_num, bool_max, c_x, c_y, c_z = c_c

            if frame_num not in self.const_frame_ind:
                self.const_frame_ind = np.append(self.const_frame_ind, frame_num)

        self.frame_traj = np.zeros((len(self.const_frame_ind), self.sample_num, 3))

        print('frames_constrained: {}'.format(self.const_frame_ind))
    def _prepare_opt(self):
        sample_num = self._order * self._sample_point + 1
        self.sample_num = sample_num

        period = 1.0/self._base_freq
        t = np.linspace(0, period, num=sample_num)

        self.H = np.zeros((self._dyn.dof * sample_num, self._dyn.base_num))

    def _obj_func(self, x):
        # objective
        q, dq, ddq = self.fourier_traj.fourier_base_x2q(x)
        # print('q:', q)
        # print('dq: ', dq)
        # print('ddq: ', ddq)

        for n in range(self.sample_num):
            vars_input = q[n, :].tolist() + dq[n, :].tolist() + ddq[n, :].tolist()
            self.H[n*self._dyn.dof:(n+1)*self._dyn.dof, :] = self._dyn.H_b_func(*vars_input)
        # print('H: ', self.H)

        f = np.linalg.cond(self.H)
        # print('f: ', f)

        # constraint
        g = [0.0] * (self._const_num * self.sample_num)
        g_cnt = 0

        # Joint constraints
        for j_c in self._joint_constraints:
            q_s, q_l, q_u, dq_l, dq_u = j_c
            co_num = self._dyn.coordinates.index(q_s)

            for qt, dqt in zip(q[:, co_num], dq[:, co_num]):
                g[g_cnt] = qt - q_u
                g_cnt += 1
                g[g_cnt] = q_l - qt
                g_cnt += 1
                g[g_cnt] = dqt - dq_u
                g_cnt += 1
                g[g_cnt] = dq_l - dqt
                g_cnt += 1
        # print('g: ', g)
        #print('constraints number: ', g_cnt)


        # Cartesian Constraints
        # print(q.shape[0])
        for c_c in self._cartesian_constraints:
            frame_num, bool_max, c_x, c_y, c_z = c_c

            for num in range(q.shape[0]):
                vars_input = q[num, :].tolist()
                p_num = self._dyn.p_n_func[frame_num](*vars_input)

                self.frame_pos[num, 0] = p_num[0, 0]
                self.frame_pos[num, 1] = p_num[1, 0]
                self.frame_pos[num, 2] = p_num[2, 0]

                if bool_max == 'max':
                    g[g_cnt] = p_num[0, 0] - c_x
                    g_cnt += 1
                    g[g_cnt] = p_num[1, 0] - c_y
                    g_cnt += 1
                    g[g_cnt] = p_num[2, 0] - c_z
                    g_cnt += 1
                elif bool_max == 'min':
                    g[g_cnt] = -p_num[0, 0] + c_x
                    g_cnt += 1
                    g[g_cnt] = -p_num[1, 0] + c_y
                    g_cnt += 1
                    g[g_cnt] = -p_num[2, 0] + c_z
                    g_cnt += 1

        fail = 1
        return f, g, fail

    def _add_obj2prob(self):
        self._opt_prob.addObj('f')

    def _add_vars2prob(self):
        joint_coef_num = 2*self._order + 1

        def rand_local(l, u, scale):
            return (np.random.random() * (u - l)/2 + (u + l)/2) * scale

        for num in range(self._dyn.dof):
            # q0
            self._opt_prob.addVar('x'+str(num*joint_coef_num + 1), 'c',
                                  lower=self._q0_min, upper=self._q0_max,
                                  value=rand_local(self._q0_min, self._q0_max, 0.1))
            # a sin
            for o in range(self._order):
                self._opt_prob.addVar('x' + str(num * joint_coef_num + 1 + o + 1), 'c',
                                      lower=self._ab_min, upper=self._ab_max,
                                      value=rand_local(self._ab_min, self._ab_max, 0.1))
            # b cos
            for o in range(self._order):
                self._opt_prob.addVar('x' + str(num * joint_coef_num + 1 + self._order + o + 1), 'c',
                                      lower=self._ab_min, upper=self._ab_max,
                                      value=rand_local(self._ab_min, self._ab_max, 0.1))

    def _add_const2prob(self):
        self._opt_prob.addConGroup('g', self._const_num * self.sample_num, type='i')

        # for c_c in self._cartesian_constraints:
        #     self._dyn.p_n[]
        # pass

    def optimize(self):
        #self._prepare_opt()
        self._opt_prob = pyOpt.Optimization('Optimial Excitation Trajectory', self._obj_func)
        self._add_vars2prob()
        self._add_obj2prob()
        self._add_const2prob()

        # print(self._opt_prob)
        #x = np.random.random((self._dyn.rbt_def.dof * (2*self._order+1)))
        #print(self._obj_func(x))
        slsqp = pyOpt.pySLSQP.SLSQP()

        slsqp.setOption('IPRINT', -1)

        [fstr, xstr, inform] = slsqp(self._opt_prob, sens_type='FD')

        self.f_result = fstr
        self.x_result = xstr

        print('Condition number: {}'.format(fstr[0]))
        print('x: {}'.format(xstr))
        #print('inform: ', inform)

        print self._opt_prob.solution(0)


    def calc_frame_traj(self):

        q, dq, ddq = self.fourier_traj.fourier_base_x2q(self.x_result)
        #print(self._dyn.p_n_func[int(self.const_frame_ind[0])])
        for i in range(len(self.const_frame_ind)):
            for num in range(q.shape[0]):
                vars_input = q[num, :].tolist()
                #print(vars_input)

                p_num = self._dyn.p_n_func[int(self.const_frame_ind[i])](*vars_input)
                #print(p_num[:, 0])
                self.frame_traj[i, num, :] = p_num[:, 0]

