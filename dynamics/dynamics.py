import sympy
from sympy import lambdify
from sympy.utilities.iterables import flatten
import numpy as np
from collections import deque
from utils import vec2so3, new_sym
from dyn_param_dep import find_dyn_parm_deps
from sympy import pprint
import time


verbose = False

if verbose:
    def vprint(*args):
        # vprint each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        for arg in args:
           print arg,
        print
else:
    vprint = lambda *a: None      # do-nothing function


class Dynamics:
    def __init__(self, rbt_def, geom, g=[0, 0, -9.81], verbose=False):
        self.rbt_def = rbt_def
        self.geom = geom
        self._g = np.matrix(g)

        self._calc_dyn()
        self._calc_regressor()
        self._calc_MCG()
        self._calc_base_param()

        print("Finished creating robot dynamics")

    def _ml2r(self, m, l):
        return sympy.Matrix(l) / m

    def _Lmr2I(self, L, m, r):
        return sympy.Matrix(L - m * vec2so3(r).transpose() * vec2so3(r))

    def _calc_dyn(self):
        print("Calculating Lagrangian...")
        # Calculate kinetic energy and potential energy
        p_e = 0
        k_e = 0
        self.k_e3 = 0

        for num in self.rbt_def.link_nums[1:]:
            k_e_n = 0
            if self.rbt_def.use_inertia[num]:
                print("Calculating the link kinetic energy of {}/{}".format(num, self.rbt_def.link_nums[-1]))
                p_e += -self.rbt_def.m[num] * self.geom.p_c[num].dot(self._g)

                k_e_n = self.rbt_def.m[num] * self.geom.v_cw[num].dot(self.geom.v_cw[num])/2 +\
                       (self.geom.w_b[num].transpose() * self.rbt_def.I_by_Llm[num] * self.geom.w_b[num])[0, 0]/2
                # if num == 5 or num == 6 or num == 7:
                #     continue

                # k_e_n = sympy.simplify(k_e_n) # this is replaced by the following code to reduce time cost
                k_e_n = sympy.factor(sympy.expand(k_e_n) - sympy.expand(k_e_n * self.rbt_def.m[num]).subs(self.rbt_def.m[num], 0)/self.rbt_def.m[num])

                # if num == 3:
                #     print(k_e_n)
                #     self.k_e3 = k_e_n

                # if num == 3:
                #     print(k_e_n)
            # if self.rbt_def.use_Ia[num]:
            #     k_m = self.rbt_def.Ia[num] * self.rbt_def.dq_for_frame[num]**2 / 2
            #     print("k_m: {}".format(k_m))
            #     k_e_n += k_m

            #vprint('k_e:', k_e_n)
            k_e += k_e_n

        # Lagrangian
        L = k_e - p_e

        # L_A, L_b = sympy.linear_eq_to_matrix([L], self.rbt_def.bary_params)
        # print("L_A: ", L_A)
        # print("L_b: ", L_b)

        tau = []
        # vprint(len(self.rbt_def.coordinates))

        print("Calculating joint torques...")
        for q, dq in zip(self.rbt_def.coordinates, self.rbt_def.d_coordinates):
            print("tau of {}".format(q.))
            dk_ddq = sympy.diff(k_e, dq)
            dk_ddq_t = dk_ddq.subs(self.rbt_def.subs_q2qt + self.rbt_def.subs_dq2dqt)
            dk_ddq_dtt = sympy.diff(dk_ddq_t, sympy.Symbol('t'))
            #print('dk_ddq_dtt:')
            #vprint(dk_ddq_dtt)
            dk_ddq_dt = dk_ddq_dtt.subs(self.rbt_def.subs_ddqt2ddq + self.rbt_def.subs_dqt2dq + self.rbt_def.subs_qt2q)
            #print('dk_ddq_dt:')
            #print(dk_ddq_dt)

            dL_dq = sympy.diff(L, q)
            #print('dL_dq:')
            #vprint(dL_dq)

            #tau.append(sympy.simplify(dk_ddq_dt - dL_dq))
            tau.append(sympy.expand(dk_ddq_dt - dL_dq))

        print("Adding frictions, motor rotor inertia and springs...")
        for i in range(self.rbt_def.frame_num):
            dq = self.rbt_def.dq_for_frame[i]

            if self.rbt_def.use_friction[i]:
                tau_f = sympy.sign(dq) * self.rbt_def.Fc[i] + dq * self.rbt_def.Fv[i] + self.rbt_def.Fo[i]
                for a in range(len(self.rbt_def.d_coordinates)):
                    dq_da = sympy.diff(dq, self.rbt_def.d_coordinates[a])
                    tau[a] += dq_da * tau_f
                    # print("dq{}_da{} = {}, tau_f = {}".format(i, a, dq_da, tau_f))

            if self.rbt_def.use_Ia[i]:
                tau_Ia = self.rbt_def.ddq_for_frame[i] * self.rbt_def.Ia[i]
                tau_index = self.rbt_def.dd_coordinates.index(self.rbt_def.ddq_for_frame[i])
                tau[tau_index] += tau_Ia
                # print("tau_Ia{}: {}".format(tau_index, tau_Ia))

        for k in range(len(self.rbt_def.K)):
            tau_k = self.rbt_def.springs[k] * self.rbt_def.K[k]
            index = self.rbt_def.coordinates.index(list(self.rbt_def.springs[k].free_symbols)[0])
            tau[index] += -tau_k

        vprint('tau: ')
        vprint(tau)

        self.tau = tau

    def _calc_regressor(self):
        print("Calculating gregressor...")
        A, b = sympy.linear_eq_to_matrix(self.tau, self.rbt_def.bary_params)
        # vprint('A:')
        # vprint(A)
        # vprint(A.shape)
        self.H = A
        # vprint('b:')
        # vprint(b)
        # vprint('Ax - b:')
        # vprint(sympy.simplify(A*sympy.Matrix(self.rbt_def.bary_params) - sympy.Matrix(self.tau)))

        input_vars = tuple(self.rbt_def.coordinates + self.rbt_def.d_coordinates + self.rbt_def.dd_coordinates)
        print('input_vars', input_vars)
        self.H_func = sympy.lambdify(input_vars, self.H)
        # vprint(self.H_func)
        # start_time = time.time()
        # vprint(self.H_func(*np.random.random_sample((len(input_vars),))))
        # vprint('time: ', time.time() - start_time)

    def _calc_base_param(self):
        print("Calculating base parameter...")
        r, P_X, P = find_dyn_parm_deps(len(self.rbt_def.coordinates), len(self.rbt_def.bary_params), self.H_func)
        self.base_num = r
        print('base parameter number: {}'.format(self.base_num))
        self.base_param = P_X.dot(np.matrix(self.rbt_def.bary_params).transpose())
        vprint('base parameters: {}'.format(self.base_param))
        vprint(P)
        P_b = P[:r].tolist()
        vprint(type(P_b))
        vprint(self.H)

        self.H_b = self.H[:, P_b]
        vprint('H_b: ', self.H_b)

        #vprint('error: ', sympy.simplify(self.H * np.matrix(self.rbt_def.bary_params).transpose() - self.H_b * self.base_param))

        input_vars = tuple(self.rbt_def.coordinates + self.rbt_def.d_coordinates + self.rbt_def.dd_coordinates)
        vprint('input_vars', input_vars)
        print("Creating H_b function...")
        self.H_b_func = sympy.lambdify(input_vars, self.H_b)

    def _calc_M(self):
        A, b = sympy.linear_eq_to_matrix(self.tau, self.rbt_def.dd_coordinates)
        vprint('dd:', self.rbt_def.dd_coordinates)
        vprint('tau:')
        vprint(A[0, :])
        vprint(A[1, :])
        self.M = A
        vprint('M:')
        vprint(self.M.shape)
        vprint(A)


    def _calc_G(self):
        subs_qdq2zero = [(dq, 0) for dq in self.rbt_def.d_coordinates]
        subs_qdq2zero += [(ddq, 0) for ddq in self.rbt_def.dd_coordinates]
        self.G = sympy.Matrix(self.tau).subs(subs_qdq2zero)
        vprint('G:')
        vprint(self.G)

    def _calc_C(self):
        subs_ddq2zero = [(ddq, 0) for ddq in self.rbt_def.dd_coordinates]
        self.C = sympy.Matrix(self.tau).subs(subs_ddq2zero) - self.G
        vprint('C:')
        vprint(self.C)

    def _calc_MCG(self):
        print("Calculating M, C and G...")
        self._calc_M()
        self._calc_G()
        self._calc_C()
