import sympy
import dh_def
import matplotlib.pyplot as plt
from frame_drawer import FrameDrawer

class Kinematics:
    def __init__(self, dh_root):
        self._dh_root = dh_root
        self._frames_drawer = FrameDrawer()

    def _cal_transfmat_iter(self, node):
        prev_link = node.get_prev_link()
        succ_link = node.get_succ_link()

        # none root link
        if prev_link is not None:
            # transformation matrix of frame
            node.cal_motion_params(prev_link.T_0n)
            print("node: {}, T: {}".format(node.get_num(), node.T_0n))

        # last link
        if len(succ_link) is 0:
            return

        for succ in succ_link:
            self._cal_transfmat_iter(succ)

    def cal_transfmats(self):
        self._cal_transfmat_iter(self._dh_root)

    def draw_frames(self):
        pass