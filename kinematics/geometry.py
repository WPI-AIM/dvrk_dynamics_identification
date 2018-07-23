import sympy
from collections import deque
import numpy as np
from robot_def import RobotDef
from frame_drawer import FrameDrawer
from utils import tranlation_transfmat, so32vec


class Geometry:
    def __init__(self, rbt_def):
        self.rbt_df = rbt_def
        self._cal_geom()
        # self._draw_geom()

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
            print('pos_c')
            self.p_c[num] = self.T_0nc[num][0:3, 3]
            print('v_cw')

            v_cw = sympy.diff(self.p_c[num].subs(self.rbt_df.subs_q2qt), t)
            v_cw = v_cw.subs(self.rbt_df.subs_dqt2dq + self.rbt_df.subs_qt2q)
            self.v_cw[num] = sympy.simplify(v_cw)

            R_t = self.R[num].subs(self.rbt_df.subs_q2qt)
            print('dR_t')
            dR_t = sympy.diff(R_t)
            print('subs dq')
            dR = dR_t.subs(self.rbt_df.subs_dqt2dq + self.rbt_df.subs_qt2q)
            #print(dR)
            # w_w = sympy.trigsimp(so32vec(dR*self.R.transpose()))
            # print('w_w: ', w_w)
            print('w_b')
            self.w_b[num] = sympy.simplify(so32vec(self.R[num].transpose() * dR))
        print('pos_c')
        print(self.p_c)
        print('v_cw')
        print(self.v_cw)
        print('w_b')
        print(self.w_b)

    def _draw_geom(self):
        frame_drawer = FrameDrawer((-0.6, 0.2), (-0.6, 0.6), (-0.6, 0.2))

        subs_q2zero = [(q, 0) for q in self.rbt_df.coordinates]

        for num in self.rbt_df.link_nums:
            T = np.matrix(self.T_0n[num].subs(subs_q2zero))
            frame_drawer.draw_frame(T, num)

            if num != 0:
                T_prev = np.matrix(self.T_0n[self.rbt_df.prev_link_num[num]].subs(subs_q2zero))
                frame_drawer.drawSegment(T_prev, T)

        frame_drawer.show()