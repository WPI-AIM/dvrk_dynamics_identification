import cvxpy as cp
import numpy as np
import sympy
import osqp
from utils import gen_DLki_mat


class SDPOpt:
    def __init__(self, W, tau, rbt_def, value_constraints=[]):
        self.small_positive_num = 0.000001
        self.min_Fc = 0.005
        self.min_Fv = 0.005
        self.min_Ia = 0.02

        self._W = W
        self._tau = tau
        self._m, self._n = self._W.shape
        self._rbt_def = rbt_def
        value_constraints_len = len(value_constraints)
        if value_constraints_len != 0:
            if value_constraints_len != self._rbt_def.frame_num - 1:
                raise ValueError("Value constraint number {} should be the same as " +
                                 "the joint number of the robot {}.".format(value_constraints_len, self._rbt_def.frame_num - 1))
            for c in value_constraints:
                if len(c) != 11:
                    raise ValueError("The constraint should be a tuple of " +
                                     "(min_m, max_m, min_x, max_x, min_y, max_y, min_z, max_z, max_Fc, max_Fv, max_Fo).")
                if c[0] < 0 or c[1] < 0:
                    raise ValueError("Mass constraints should be positive.")

        self._value_constraints = value_constraints

        print("Regressor shape: {}".format(W.shape))
        print("Regressand shape: {}".format(tau.shape))

    def _create_var(self):
        print("Creating variables...")
        self._x = cp.Variable(self._n)

    def _create_obj(self):
        print("Creating optimization objective...")
        self._obj = cp.Minimize(cp.sum_squares(self._W * self._x - self._tau))

    def _create_constraints(self):
        self._constraints = []

        print("Creating constraints...")

        DLkis = gen_DLki_mat()

        i_param = 0
        i_link = 0

        for f in range(self._rbt_def.frame_num)[1:]:
            if self._rbt_def.use_inertia[f]:
                # semi-definite
                D = np.zeros((6, 6))
                for i in range(10):
                    D += DLkis[i] * self._x[i_param + i]
                self._constraints.append(D >> np.identity(6) * self.small_positive_num)

            #if len(self._value_constraints) != 0:

            # constraint order: (min_m, max_m, min_x, max_x, min_y, max_y, min_z, max_z)
            if len(self._value_constraints) != 0:
                min_m, max_m, min_x, max_x, min_y, max_y, min_z, max_z, max_Fc, max_Fv, max_Fo = self._value_constraints[i_link]

                if self._rbt_def.use_inertia[f]:
                    # mass of center position
                    # x
                    self._constraints.append(self._x[i_param + 6] >= min_x * self._x[i_param + 9])
                    self._constraints.append(self._x[i_param + 6] <= max_x * self._x[i_param + 9])
                    # y
                    self._constraints.append(self._x[i_param + 7] >= min_y * self._x[i_param + 9])
                    self._constraints.append(self._x[i_param + 7] <= max_y * self._x[i_param + 9])
                    # z
                    self._constraints.append(self._x[i_param + 8] >= min_z * self._x[i_param + 9])
                    self._constraints.append(self._x[i_param + 8] <= max_z * self._x[i_param + 9])

                    # mass
                    self._constraints.append(self._x[i_param + 9] >= min_m)
                    self._constraints.append(self._x[i_param + 9] <= max_m)

                    i_param += 10

                if self._rbt_def.use_friction[f]:
                    # Coulomb friction
                    if 'Coulomb' in self._rbt_def.friction_type:
                        self._constraints.append(self._x[i_param] >= self.min_Fc)
                        self._constraints.append(self._x[i_param] <= max_Fc)
                        i_param += 1

                    # Viscous friction
                    if 'viscous' in self._rbt_def.friction_type:
                        self._constraints.append(self._x[i_param] >= self.min_Fv)
                        self._constraints.append(self._x[i_param] <= max_Fv)
                        i_param += 1

                    # Coulomb friction offset
                    if 'offset' in self._rbt_def.friction_type:
                        self._constraints.append(self._x[i_param] <= max_Fo)
                        self._constraints.append(self._x[i_param] >= -max_Fo)
                        i_param += 1

                # Inertia of motor
                if self._rbt_def.use_Ia[f]:
                    print("Ia{} param{}".format(f, i_param+1))
                    self._constraints.append(self._x[i_param] >= self.min_Ia)
                    i_param += 1

            else:
                if self._rbt_def.use_inertia[f]:
                    i_param += 10

                # Coulomb friction
                if 'Coulomb' in self._rbt_def.friction_type:
                    i_param += 1

                # Viscous friction
                if 'viscous' in self._rbt_def.friction_type:
                    i_param += 1

                # Coulomb friction offset
                if 'offset' in self._rbt_def.friction_type:
                    i_param += 1

                # Inertia of motor
                if self._rbt_def.use_Ia[f]:
                    i_param += 1

    def solve(self):
        self._create_var()
        self._create_obj()
        self._create_constraints()

        print("Solving problem...")
        self._prob = cp.Problem(self._obj, self._constraints)

        result = self._prob.solve(solver=cp.SCS)

        self.x_result = self._x.value
        print(self._x.value)




# m = 30000
# n = 100
# np.random.seed(1)
# A = np.random.randn(m, n)
# b = np.random.randn(m)
#
# # Construct the problem.
# x = cp.Variable(n)
# objective = cp.Minimize(cp.sum_squares(A*x - b))
# constraints = [x <= 1, x >= -1]
# prob = cp.Problem(objective, constraints)
#
# # The optimal objective value is returned by `prob.solve()`.
# result = prob.solve(solver=cp.OSQP)
# # The optimal value for x is stored in `x.value`.
# print(x.value)
# # The optimal Lagrange multiplier for a constraint is stored in
# # `constraint.dual_value`.
# print(constraints[0].dual_value)