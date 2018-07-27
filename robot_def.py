from dh_def import DHDef
import sympy
from sympy.physics.vector import dynamicsymbols
import numpy as np
from utils import inertia_vec2tensor, ml2r, Lmr2I, new_sym


def new_sym(name):
    return sympy.symbols(name, real=True)

_cos = sympy.cos
_sin = sympy.sin

_dh_alpha, _dh_a, _dh_d, _dh_theta = new_sym('alpha,a,d,theta')
default_dh_symbols = (_dh_alpha, _dh_a, _dh_d, _dh_theta)

_standard_dh_transfmat = sympy.Matrix([
    [_cos(_dh_theta), -_sin(_dh_theta) * _cos(_dh_alpha), _sin(_dh_theta) * _sin(_dh_alpha), _dh_a * _cos(_dh_theta)],
    [_sin(_dh_theta),  _cos(_dh_theta) * _cos(_dh_alpha), -_cos(_dh_theta) * _sin(_dh_alpha), _dh_a * _sin(_dh_theta)],
    [0, _sin(_dh_alpha), _cos(_dh_alpha), _dh_d],
    [0, 0, 0, 1]])

_modified_dh_transfmat = sympy.Matrix([
    [_cos(_dh_theta), -_sin(_dh_theta), 0, _dh_a],
    [_sin(_dh_theta) * _cos(_dh_alpha), _cos(_dh_theta) * _cos(_dh_alpha), -_sin(_dh_alpha), -_sin(_dh_alpha) * _dh_d],
    [_sin(_dh_theta) * _sin(_dh_alpha), _cos(_dh_theta) * _sin(_dh_alpha), _cos(_dh_alpha), _cos(_dh_alpha) * _dh_d],
    [0, 0, 0, 1]])

_friction_types = ['Coulomb', 'viscous', 'offset']

verbose = False

if verbose:
    def vprint(*args):
        # Print each argument separately so caller doesn't need to
        # stuff everything to be printed into a single string
        for arg in args:
           print arg,
        print
else:
    vprint = lambda *a: None      # do-nothing function

class RobotDef:
    def __init__(self, params, dh_convention='mdh', friction_type=['viscous']):

        self.frame_num = len(params)
        self.link_nums = [p[0] for p in params]
        self.prev_link_num = [p[1] for p in params]
        self.succ_link_num = [p[2] for p in params]
        self.dh_a = [p[3] for p in params]
        self.dh_alpha = [p[4] for p in params]
        self.dh_d = [p[5] for p in params]
        self.dh_theta = [p[6] for p in params]
        self.dh_convention = dh_convention
        if self.dh_convention in ['sdh', 'std']:
            self._dh_transmat = _standard_dh_transfmat
        elif self.dh_convention in ['mdh', 'modified']:
            self._dh_transmat = _modified_dh_transfmat
        self.friction_type = friction_type
        vprint("frame_number: ", self.frame_num)
        vprint(self.succ_link_num)
        vprint(sympy.Matrix(self.dh_a + self.dh_alpha + self.dh_d + self.dh_theta).free_symbols)
        #print(sympy.Matrix(self.dh_a + self.dh_alpha, self.dh_d + self.dh_theta))
        #for p in dh_params:

        self._gen_dh_transfm()
        self._gen_params()
        self._dyn_params()
        self._gen_coordinates()

    def _gen_coordinates(self):
        self.coordinates = []
        # self.joint_coordinate = list(range(self.frame_num))
        for num in self.link_nums:
            for s in self.dh_T[num].free_symbols:
                if s not in self.coordinates:
                    self.coordinates += [s]
        self.dof = len(self.coordinates)
        print(self.coordinates)

        self.d_coordinates = [new_sym('d'+co.name) for co in self.coordinates]
        self.dd_coordinates = [new_sym('dd' + co.name) for co in self.coordinates]
        vprint(self.d_coordinates)
        vprint(self.dd_coordinates)

        self.coordinates_t = [dynamicsymbols(co.name+'t') for co in self.coordinates]
        vprint(self.coordinates_t)
        self.d_coordinates_t = [sympy.diff(co_t) for co_t in self.coordinates_t]
        vprint(self.d_coordinates_t)
        self.dd_coordinates_t = [sympy.diff(d_co_t) for d_co_t in self.d_coordinates_t]
        vprint(self.dd_coordinates_t)

        self.subs_q2qt = [(q, qt) for q, qt in zip(self.coordinates, self.coordinates_t)]
        vprint(self.subs_q2qt)
        self.subs_dq2dqt = [(dq, dqt) for dq, dqt in zip(self.d_coordinates, self.d_coordinates_t)]
        vprint(self.subs_dq2dqt)
        self.subs_ddq2ddqt = [(ddq, ddqt) for ddq, ddqt in zip(self.dd_coordinates, self.dd_coordinates_t)]
        vprint(self.subs_ddq2ddqt)
        self.subs_qt2q = [(qt, q) for q, qt in zip(self.coordinates, self.coordinates_t)]
        vprint(self.subs_qt2q)
        self.subs_dqt2dq = [(dqt, dq) for dq, dqt in zip(self.d_coordinates, self.d_coordinates_t)]
        vprint(self.subs_dqt2dq)
        self.subs_ddqt2ddq = [(ddqt, ddq) for ddq, ddqt in zip(self.dd_coordinates, self.dd_coordinates_t)]
        vprint(self.subs_ddq2ddqt)

    def _gen_dh_transfm(self):
        self.dh_T = []
        for num in self.link_nums:
            subs_dh = [(_dh_alpha, self.dh_alpha[num]), (_dh_a, self.dh_a[num]), (_dh_d, self.dh_d[num]), (_dh_theta, self.dh_theta[num])]
            self.dh_T.append(self._dh_transmat.subs(subs_dh))
        #print(self.dh_T)

    def _gen_params(self):
        self.std_params = []
        self.bary_params = []

        self.m = list(range(self.frame_num))
        self.l = list(range(self.frame_num))
        self.r = list(range(self.frame_num))
        self.r_by_ml = list(range(self.frame_num))
        self.L_vec = list(range(self.frame_num))
        self.I_vec = list(range(self.frame_num))
        self.L_mat = list(range(self.frame_num))
        self.I_mat = list(range(self.frame_num))
        self.I_by_Llm = list(range(self.frame_num))
        self.Fc = list(range(self.frame_num))
        self.Fv = list(range(self.frame_num))
        self.Fo = list(range(self.frame_num))

        for num in self.link_nums[1:]:
            self.m[num] = new_sym('m'+str(num))
            self.l[num] = [new_sym('l'+str(num)+dim) for dim in ['x', 'y', 'z']]
            self.r[num] = [new_sym('r'+str(num)+dim) for dim in ['x', 'y', 'z']]
            self.I_vec[num] = [new_sym('I'+str(num)+elem) for elem in ['xx', 'xy', 'xz', 'yy', 'yz', 'zz']]
            self.L_vec[num] = [new_sym('L'+str(num)+elem) for elem in ['xx', 'xy', 'xz', 'yy', 'yz', 'zz']]

            self.I_mat[num] = inertia_vec2tensor(self.I_vec[num])
            self.L_mat[num] = inertia_vec2tensor(self.L_vec[num])

            self.r_by_ml[num] = ml2r(self.m[num], self.l[num])
            self.I_by_Llm[num] = Lmr2I(self.L_mat[num], self.m[num], self.r_by_ml[num])

            if 'Coulomb' in self.friction_type:
                self.Fc[num] = new_sym('Fc' + str(num))
            if 'viscous' in self.friction_type:
                self.Fv[num] = new_sym('Fv' + str(num))
            if 'offset' in self.friction_type:
                self.Fo[num] = new_sym('Fo' + str(num))

        vprint(self.m)
        vprint(self.l)
        vprint(self.r)
        vprint(self.I_vec, self.L_vec)

    def _dyn_params(self):
        self.params = []

        for num in self.link_nums[1:]:
            self.params += self.L_vec[num]
            self.params += self.l[num]
            self.params += [self.m[num]]

            if 'Coulomb' in self.friction_type:
                self.params += [self.Fc[num]]
            if 'viscous' in self.friction_type:
                self.params += [self.Fv[num]]
            if 'offset' in self.friction_type:
                self.params += [self.Fo[num]]

        vprint(self.params)
