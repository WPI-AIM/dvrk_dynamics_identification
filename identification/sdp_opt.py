import cvxpy as cp
import numpy as np
import sympy
import osqp
from utils import gen_DLki_mat


small_positive_num = 0.00001


class SDPOpt:
    def __init__(self, W, tau, params, dof, value_constraints=[]):
        self._W = W
        self._tau = tau
        self._m, self._n = self._W.shape
        self._param = params
        self._dof = dof
        value_constraints_len = len(value_constraints)
        if value_constraints_len != 0:
            if value_constraints_len != dof:
                raise ValueError("Value constraint number {} should be the same as the DoF of the robot {}.".format(
                    value_constraints_len, self._dof))
            for c in value_constraints:
                if len(c) != 8:
                    raise ValueError("The constraint should be a tuple of " +
                                     "(min_m, max_m, min_x, max_x, min_y, max_y, min_z, max_z).")
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
        self._obj = cp.Minimize(cp.sum_squares(self._W*self._x - self._tau))

    def _create_constraints(self):
        self._constraints = []

        print("Creating constraints...")

        DLkis = gen_DLki_mat()

        for d in range(self._dof):
            # semi-definite
            D = np.zeros((6, 6))
            for i in range(10):
                D += DLkis[i] * self._x[d * 10 + i]

            self._constraints.append(D >> np.identity(6) * small_positive_num)

            if len(self._value_constraints) != 0:
                # constraint order: (min_m, max_m, min_x, max_x, min_y, max_y, min_z, max_z)
                min_m, max_m, min_x, max_x, min_y, max_y, min_z, max_z = self._value_constraints[d]

                # mass
                self._constraints.append(self._x[d * 10 + 9] >= min_m)
                self._constraints.append(self._x[d * 10 + 9] <= max_m)

                # mass of center position
                # x
                self._constraints.append(self._x[d * 10 + 6] >= min_x * self._x[d * 10 + 9])
                self._constraints.append(self._x[d * 10 + 6] <= max_x * self._x[d * 10 + 9])
                # y
                self._constraints.append(self._x[d * 10 + 7] >= min_y * self._x[d * 10 + 9])
                self._constraints.append(self._x[d * 10 + 7] <= max_y * self._x[d * 10 + 9])
                # z
                self._constraints.append(self._x[d * 10 + 8] >= min_z * self._x[d * 10 + 9])
                self._constraints.append(self._x[d * 10 + 8] <= max_z * self._x[d * 10 + 9])



    def solve(self):
        self._create_var()
        self._create_obj()
        self._create_constraints()

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