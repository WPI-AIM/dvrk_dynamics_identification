import numpy as np
from scipy import linalg, hstack

def find_dyn_parm_deps(dof, parm_num, regressor_func):
    '''
    Find dynamic parameter dependencies (i.e., regressor column dependencies).
    '''

    verbose = False

    if verbose:
        def vprint(*args):
            # Print each argument separately so caller doesn't need to
            # stuff everything to be printed into a single string
            for arg in args:
                print arg,
            print
    else:
        vprint = lambda *a: None  # do-nothing function


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
    vprint('rank: ', r)
    _, _, P = linalg.qr(Z, pivoting=True)
    vprint(P)

    Q, R = linalg.qr(Z[:, P])
    R1 = R[:r, :r]
    R2 = R[:r, r:]
    vprint(np.eye(r))
    vprint('inv_R1 * R2: ', np.matmul(linalg.inv(R1), R2))
    vprint('T_P', np.transpose(np.eye(parm_num)[:, P]))
    vprint('right', hstack((np.eye(r), np.matmul(linalg.inv(R1), R2))))
    P_X = np.matmul(hstack((np.eye(r), np.matmul(linalg.inv(R1), R2))), np.transpose(np.eye(parm_num)[:, P]))

    vprint(P_X)
    X = np.random.random_sample((parm_num,))
    XB1 = P_X.dot(X)
    vprint('error:', np.matmul(Z, X) - np.matmul(Z[:,P[:r]], XB1))

    return r, P_X, P