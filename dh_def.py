import sympy
from sympy.physics.vector import dynamicsymbols
from utils import vec2so3, inertia_vec2tensor, tranlation_transfmat, so32vec

def new_sym(name):
    return sympy.symbols(name, real=True)

_cos = sympy.sin
_sin = sympy.cos

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



class DHDef:
    def __init__(self, num, joint_type, a, alpha, d, theta, dh_type, prev, succ,
                 active=True, fric_add_to=None, fric_types = _friction_types):
        print('Creating frame: ' + str(num))
        self._num = num
        if joint_type == 'R':
            print('Revolute joint')
            self._revolute = True
        elif joint_type == 'P':
            print('Prismatic joint')
            self._revolute = False
        else:
            raise ValueError("Joint type can only be 'R' or 'P'")
        self._a = a
        self._alpha = alpha
        self._d = d
        self._theta = theta

        self._prev_link = prev
        self._succ_link = succ

        sdh_str = ['sdh', 'std']
        mdh_str = ['mdh', 'modified']
        self._dh_symbols = default_dh_symbols
        if dh_type in sdh_str:
            self._using_sdh = True
            self._dh_transfmat = _standard_dh_transfmat
            print('Using standard DH')
        elif dh_type in mdh_str:
            self._using_sdh = False
            self._dh_transfmat = _modified_dh_transfmat
            print('Using modified DH')
        else:
            raise ValueError("You can choose DH type in either {} or {}".format(sdh_str, mdh_str))

        self._active = active
        if not self._active:
            self._fric_add_to = fric_add_to

        self._fric_types = fric_types

        self.T_0n = sympy.eye(4)
        self.dh_T = sympy.eye(4)
        self.T_0nc = sympy.eye(4)
        self.pos_c = sympy.ZeroMatrix(3, 1)
        self.R = sympy.eye(3)

        self.w_b = sympy.ZeroMatrix(3, 1)
        self.v_cb = sympy.ZeroMatrix(3, 1)

        if self._prev_link is not None:
            self._get_all_coordinates()
            self._gen_dyn_vars()
            self._cal_transfmat()

    def set_succ(self, succs):
        self._succ_link = succs

    def append_succ(self, succ):
        self._succ_link.append(succ)

    def _gen_dyn_vars(self):
        self.std_params = []
        self.bary_params = []

        self._m = new_sym('m'+str(self._num))
        self._l = sympy.Matrix([new_sym('l'+str(self._num)+dim) for dim in ['x', 'y', 'z']])
        self._r = sympy.Matrix([new_sym('r'+str(self._num)+dim) for dim in ['x', 'y', 'z']])

        self._I_vec = [new_sym('I'+str(self._num)+elem) for elem in ['xx', 'xy', 'xz', 'yy', 'yz', 'zz']]
        self._L_vec = [new_sym('L'+str(self._num)+elem) for elem in ['xx', 'xy', 'xz', 'yy', 'yz', 'zz']]

        self._I_mat = inertia_vec2tensor(self._I_vec)
        self._L_mat = inertia_vec2tensor(self._L_vec)

        #self.params += self._m + self._l + self.

        if 'Coulomb' in self._fric_types:
            self._Fc = new_sym('Fc'+str(self._num))
        if 'viscous' in self._fric_types:
            self._Fv = new_sym('Fv'+str(self._num))
        if 'Coulomb' in self._fric_types:
            self._Fo = new_sym('Fo'+str(self._num))

    def _cal_transfmat(self):
        self.dh_T = self._dh_transfmat.subs([(self._dh_symbols[0], self._alpha),
                                          (self._dh_symbols[1], self._a),
                                          (self._dh_symbols[2], self._d),
                                          (self._dh_symbols[3], self._theta)])

    def cal_motion_params(self, pre_T):
        # transformation of current frame
        self.T_0n = sympy.simplify(pre_T * self.dh_T)
        # transformation of COM
        self.T_0nc = sympy.sympify(self.T_0n * tranlation_transfmat(self._r))
        self.pos_c = self.T_0nc[0:3, 3]
        self.R = self.T_0n[0:3, 0:3]

        q_subs_cells = [(q, qt) for q, qt in zip(self._coordinates, self._coordinates_t)]
        q_subs_back_cells = [(qt, q) for q, qt in zip(self._coordinates, self._coordinates_t)]
        dq_subs_cells = [(dq, dqt) for dq, dqt in zip(self._d_coordinates, self._d_coordinates_t)]
        dq_subs_back_cells = [(qt, q) for q, qt in zip(self._d_coordinates, self._d_coordinates_t)]

        print('pos_c: ', self.pos_c)
        t = sympy.symbols('t')
        self.v_cb = sympy.diff(self.pos_c.subs(q_subs_cells), t)
        self.v_cb = self.v_cb.subs(dq_subs_back_cells)
        self.v_cb = self.v_cb.subs(q_subs_back_cells)
        print('v_cb: ', self.v_cb)

        R_t = self.R.subs(q_subs_cells)
        print('dR_t')
        dR_t = sympy.diff(R_t)
        print('subs dq')
        dR = dR_t.subs(dq_subs_back_cells)
        print('subs q')
        dR = dR.subs(q_subs_back_cells)
        #print(dR)
        # w_w = sympy.trigsimp(so32vec(dR*self.R.transpose()))
        # print('w_w: ', w_w)
        print('w_b')
        self.w_b = so32vec(self.R.transpose() * dR)
        print(self.w_b)


    def _get_all_coordinates(self):
        coordinates = []

        link = self
        while link._prev_link != None:
            syms = []
            if self._revolute:
                syms = link._theta.free_symbols
            else:
                syms = link._d.free_symbols

            for s in syms:
                if s not in coordinates:
                    coordinates.append(s)
            link = link._prev_link

        print('coodirnates: ', coordinates)
        self._coordinates = coordinates
        self._d_coordinates = [new_sym('d'+co.name) for co in self._coordinates]
        self._dd_coordinates = [new_sym('dd' + co.name) for co in self._coordinates]
        #print(self._d_coordinates)
        #print(self._dd_coordinates)

        self._coordinates_t = [dynamicsymbols(co.name+'t') for co in self._coordinates]
        #print(self._coordinates_t)
        self._d_coordinates_t = [sympy.diff(co_t) for co_t in self._coordinates_t]
        #print(self._d_coordinates_t)
        self._dd_coordinates_t = [sympy.diff(d_co_t) for d_co_t in self._d_coordinates_t]
        #print(self._dd_coordinates_t)


    def get_prev_link(self):
        return self._prev_link

    def get_succ_link(self):
        return self._succ_link

    def get_num(self):
        return self._num