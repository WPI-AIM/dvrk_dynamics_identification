import numpy as np
from scipy import linalg, hstack

def find_dyn_parm_deps(dof, parm_num, regressor_func):
    '''
    Find dynamic parameter dependencies (i.e., regressor column dependencies).
    '''

    samples = parm_num*2
    round = 10

    pi = np.pi

    Z = np.zeros((dof * samples, parm_num))

    for i in range(samples):
        q = (np.random.random_sample((dof,)) * 2.0 * pi - pi).tolist()
        dq = (np.random.random_sample((dof,)) * 2.0 * pi - pi).tolist()
        ddq = (np.random.random_sample((dof,)) * 2.0 * pi - pi).tolist()
        input_vars = q + dq + ddq
        Z[i * dof: i * dof + dof, :] = np.matrix(regressor_func(*input_vars))

    r = np.linalg.matrix_rank(Z)
    _, _, P = linalg.qr(Z, pivoting=True)

    Q, R = linalg.qr(Z[:, P])
    R1 = R[:r, :r]
    R2 = R[:r, r:]
    P_X = np.matmul(hstack((np.eye(r), np.matmul(linalg.inv(R1), R2))), np.transpose(np.eye(parm_num)[:, P]))

    P_X = np.around(P_X, 8)

    return r, P_X, P