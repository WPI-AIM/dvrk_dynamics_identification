import cvxpy as cp
import numpy as np
import sympy
import osqp

class SDPOpt:
    def __init__(self, W, tau, params):
        self._W = W
        self._tau = tau
        self._m, self._n = self._W.shape

        print(W.shape)
        print(tau.shape)

    def _create_var(self):
        self._x = cp.Variable(self._n)

    def _create_obj(self):
        self._obj = cp.Minimize(cp.sum_squares(self._W*self._x - self._tau))

    def _create_constraints(self):
        self._constraints = []
        # for i in range(self._n-2):
        #     self._constraints.append(self._x[i] >= -10)
            #self._constraints.append(self._x[i] <= 1)

    def solve(self):
        self._create_var()
        self._create_obj()
        self._create_constraints()

        self._prob = cp.Problem(self._obj, self._constraints)

        result = self._prob.solve(solver=cp.OSQP)

        print(self._x.value)




m = 30000
n = 100
np.random.seed(1)
A = np.random.randn(m, n)
b = np.random.randn(m)

# Construct the problem.
x = cp.Variable(n)
objective = cp.Minimize(cp.sum_squares(A*x - b))
constraints = [x <= 1, x >= -1]
prob = cp.Problem(objective, constraints)

# The optimal objective value is returned by `prob.solve()`.
result = prob.solve(solver=cp.OSQP)
# The optimal value for x is stored in `x.value`.
print(x.value)
# The optimal Lagrange multiplier for a constraint is stored in
# `constraint.dual_value`.
print(constraints[0].dual_value)