import sympy



def vec2so3(vec):
    return sympy.Matrix([[0,        -vec[2],    vec[1]],
                         [vec[2],   0,          -vec[0]],
                         [-vec[1],  vec[0],     0]])


def so32vec(mat):
    return sympy.Matrix([[mat[2,1]],
                         [mat[0,2]],
                         [mat[1,0]]])


def inertia_vec2tensor(vec):
    return sympy.Matrix([[vec[0], vec[1], vec[2]],
                         [vec[1], vec[3], vec[4]],
                         [vec[2], vec[4], vec[5]]])


def tranlation_transfmat(v):
    return sympy.Matrix([[1, 0, 0, v[0]],
                        [0, 1, 0, v[1]],
                        [0, 0, 1, v[2]],
                        [0, 0, 0, 1]])