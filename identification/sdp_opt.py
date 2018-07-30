import cvxpy as cp
import numpy as np
import sympy
import osqp
from utils import gen_DLki_mat


small_positive_num = 0.00000001


class SDPOpt:
    def __init__(self, W, tau, params, dof, constraints=[]):
        self._W = W
        self._tau = tau
        self._m, self._n = self._W.shape
        self._param = params
        self._dof = dof

        print(W.shape)
        print(tau.shape)

    def _create_var(self):
        self._x = cp.Variable(self._n)

    def _create_obj(self):
        self._obj = cp.Minimize(cp.sum_squares(self._W*self._x - self._tau))

    def _create_constraints(self):
        self._constraints = []

        DLkis = gen_DLki_mat()
        Ds = []

        for d in range(self._dof):
            D = np.zeros((6, 6))
            #D1 = sympy.Matrix(np.zeros((6, 6)))
            for i in range(10):
                D += DLkis[i] * self._x[d * 10 + i]
                # += sympy.Matrix(DLkis[i]) * self._param[d * 10 + i]
            #print(D1)
            # semi-definite constraint
            self._constraints.append(D >> np.identity(6) * small_positive_num)
            # reasonable


        # i = 0
        # while i < self._n:

            # generalized_inertia = cp.Variable((6, 6))
            #
            # # symetric
            # self._constraints.append(generalized_inertia == generalized_inertia.T)
            #
            # # L
            # self._constraints.append(generalized_inertia[0, 0] == self._x[i])
            # self._constraints.append(generalized_inertia[0, 1] == self._x[i + 1])
            # self._constraints.append(generalized_inertia[0, 2] == self._x[i + 2])
            # self._constraints.append(generalized_inertia[1, 1] == self._x[i + 3])
            # self._constraints.append(generalized_inertia[1, 2] == self._x[i + 4])
            # self._constraints.append(generalized_inertia[2, 2] == self._x[i + 5])
            #
            # # l
            # self._constraints.append(generalized_inertia[0, 3] == 0)
            # self._constraints.append(generalized_inertia[0, 4] == self._x[i + 8])
            # self._constraints.append(generalized_inertia[0, 5] == -self._x[i + 7])
            # self._constraints.append(generalized_inertia[1, 3] == -self._x[i + 8])
            # self._constraints.append(generalized_inertia[1, 4] == 0)
            # self._constraints.append(generalized_inertia[1, 5] == self._x[i + 6])
            # self._constraints.append(generalized_inertia[2, 3] == self._x[i + 7])
            # self._constraints.append(generalized_inertia[2, 4] == -self._x[i + 6])
            # self._constraints.append(generalized_inertia[2, 5] == 0)
            #
            # # m
            # self._constraints.append(generalized_inertia[3, 3] == self._x[i + 9])
            # self._constraints.append(generalized_inertia[3, 4] == 0)
            # self._constraints.append(generalized_inertia[3, 5] == 0)
            # self._constraints.append(generalized_inertia[4, 4] == self._x[i + 9])
            # self._constraints.append(generalized_inertia[4, 5] == 0)
            # self._constraints.append(generalized_inertia[5, 5] == self._x[i + 9])
            #
            # # Semi-definite
            # self._constraints.append(generalized_inertia >> np.identity(6)*small_positive_num)

            # i += 10


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