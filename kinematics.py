import sympy
import dh_def
import matplotlib.pyplot as plt
from frame_drawer import FrameDrawer
import numpy as np
from collections import deque


class Kinematics:
    def __init__(self, dh_root):
        self._dh_root = dh_root

    def _cal_transfmat_iter(self, node):
        prev_link = node.get_prev_link()
        succ_link = node.get_succ_link()

        # none root link
        if prev_link is not None:
            # transformation matrix of frame
            node.cal_motion_params(prev_link.T_0n)
            print("node: {}".format(node.get_num()))

        # last link
        if len(succ_link) is 0:
            return

        for succ in succ_link:
            self._cal_transfmat_iter(succ)

    def cal_transfmats(self):
        self._cal_transfmat_iter(self._dh_root)

    def draw_frames(self):
        frame_drawer = FrameDrawer((-0.6, 0.2), (-0.6, 0.6), (-0.6, 0.2))

        node_q = deque([self._dh_root])
        cnt = 0
        while len(node_q) != 0:
            node = node_q.pop()
            T = None
            if node.get_prev_link() is None:
                T = np.matrix(node.T_0n)
            else:
                co_subs = [(var, 0) for var in node._coordinates]
                T = np.matrix(node.T_0n.subs(co_subs))
            print('frame: ', node._num)

            frame_drawer.draw_frame(T, node._num)
            if node.get_prev_link() is not None:
                T_prev = None
                if node.get_prev_link().get_prev_link() is None:
                    T_prev = np.matrix(node.get_prev_link().T_0n)
                else:
                    co_subs = [(var, 0) for var in node.get_prev_link()._coordinates]
                    T_prev = np.matrix(node.get_prev_link().T_0n.subs(co_subs))
                frame_drawer.drawSegment(T_prev, T)
                print('link: ', node.get_prev_link()._num, 'to', node._num)

            for s in node.get_succ_link():
                node_q.append(s)

            cnt += 1
            if cnt > 4:
                #break
                pass
        frame_drawer.show()