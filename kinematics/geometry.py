import sympy
from collections import deque
import numpy as np
from robot_def import RobotDef
from frame_drawer import FrameDrawer
from utils import tranlation_transfmat, so32vec


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


class Geometry:
    def __init__(self, rbt_def):
        self.rbt_df = rbt_def
        self._cal_geom()
        self._calc_functions()
        # self.draw_geom()

    def _cal_geom(self):
        self.T_0n = list(range(self.rbt_df.frame_num))
        self.p_n = list(range(self.rbt_df.frame_num))
        self.T_0nc = list(range(self.rbt_df.frame_num))
        self.p_c = list(range(self.rbt_df.frame_num))
        self.R = list(range(self.rbt_df.frame_num))
        self.v_cw = list(range(self.rbt_df.frame_num))
        self.w_b = list(range(self.rbt_df.frame_num))

        t = sympy.symbols('t')

        for num in self.rbt_df.link_nums:
            if num == 0:
                self.T_0n[num] = self.rbt_df.dh_T[num]
                continue
            self.T_0n[num] = self.T_0n[self.rbt_df.prev_link_num[num]] * self.rbt_df.dh_T[num]
            self.R[num] = self.T_0n[num][0:3, 0:3]
            self.p_n[num] = self.T_0n[num][0:3, 3]
            self.T_0nc[num] = sympy.sympify(self.T_0n[num] * tranlation_transfmat(self.rbt_df.r_by_ml[num]))
            vprint('pos_c{}'.format(num))
            self.p_c[num] = self.T_0nc[num][0:3, 3]
            vprint('v_cw{}'.format(num))


        #TO MAKE FASTER : COMMENT THIS
            v_cw = sympy.diff(self.p_c[num].subs(self.rbt_df.subs_q2qt), t)
            v_cw = v_cw.subs(self.rbt_df.subs_dqt2dq + self.rbt_df.subs_qt2q)
            self.v_cw[num] = sympy.simplify(v_cw)

            R_t = self.R[num].subs(self.rbt_df.subs_q2qt)
            vprint('dR_t{}'.format(num))
            dR_t = sympy.diff(R_t)
            vprint('subs dq{}'.format(num))
            dR = dR_t.subs(self.rbt_df.subs_dqt2dq + self.rbt_df.subs_qt2q)
            #print(dR)
            # w_w = sympy.trigsimp(so32vec(dR*self.R.transpose()))
            # print('w_w: ', w_w)
            vprint('w_b')
            self.w_b[num] = sympy.simplify(so32vec(self.R[num].transpose() * dR))
        vprint('pos_c')
        vprint(self.p_c)
        vprint('v_cw')
        vprint(self.v_cw)
        vprint('w_b')
        vprint(self.w_b)

    def _calc_functions(self):
        self.p_n_func = ["" for x in range(self.rbt_df.frame_num)]
        #self.p_n_func = np.zeros(self.rbt_df.dof)
        input_vars = tuple(self.rbt_df.coordinates)

        for num in range(self.rbt_df.frame_num):
            self.p_n_func[num] = sympy.lambdify(input_vars, self.p_n[num])

    def draw_geom(self, angle=0):
        frame_drawer = FrameDrawer((-0.6, 0.4), (-0.4, 0.4), (-0.6, 0.2))

        if angle == 0:
            subs_q2zero = [(q, angle) for q in self.rbt_df.coordinates]

        else :
            subs_q2zero = []
            x = self.rbt_df.coordinates
            for i in range(len(x)):
                subs_q2zero.append((x[i], angle[i]))

        for num in self.rbt_df.link_nums:
            T = np.matrix(self.T_0n[num].subs(subs_q2zero))
            frame_drawer.draw_frame(T, num)
            #print(T[0:3, 3])
            if num != 0:
                T_prev = np.matrix(self.T_0n[self.rbt_df.prev_link_num[num]].subs(subs_q2zero))
                frame_drawer.drawSegment(T_prev, T)

        frame_drawer.show()


