import numpy as np
import matplotlib.pyplot as plt


# linestyles = [('solid',               (0, ())),
#               ('densely dashed',      (0, (5, 1))),
#               ('densely dotted', (0, (1, 1))),
#
#               ('loosely dashed', (0, (5, 10))),
#               ('dashed', (0, (5, 5))),
#
#               ('loosely dashdotted', (0, (3, 10, 1, 10))),
#               ('dashdotted', (0, (3, 5, 1, 5))),
#               ('densely dashdotted', (0, (3, 1, 1, 1))),
#
#               ('loosely dashdotdotted', (0, (3, 10, 1, 10, 1, 10))),
#               ('dashdotdotted', (0, (3, 5, 1, 5, 1, 5))),
#               ('densely dashdotdotted', (0, (3, 1, 1, 1, 1, 1))),
#               ('loosely dotted',      (0, (1, 10))),
#               ('dotted',              (0, (1, 5)))]

linestyles = [('solid',               (0, ())),
              ('solid',      (0, ())),
              ('solid', (0, ())),

              ('solid', (0, ())),
              ('solid', (0, ())),

              ('solid', (0, ())),
              ('solid', (0, ())),
              ('solid', (0, ())),

              ('loosely dashdotdotted', (0, (3, 10, 1, 10, 1, 10))),
              ('dashdotdotted', (0, (3, 5, 1, 5, 1, 5))),
              ('densely dashdotdotted', (0, (3, 1, 1, 1, 1, 1))),
              ('loosely dotted',      (0, (1, 10))),
              ('dotted',              (0, (1, 5)))]


class TrajPlotter:
    def __init__(self, fourier_traj, frame_traj = [], const_frame_num = []):
        self._fourier_traj = fourier_traj
        self._frame_traj = frame_traj
        self._const_frame_ind = const_frame_num
        #print(self._frame_traj)

    def plot_desired_traj(self, fourier_x):
        x = self._fourier_traj.t

        q, dq, ddq = self._fourier_traj.fourier_base_x2q(fourier_x)

        fig = plt.figure(1)
        plt_q = fig.add_subplot(311)
        plt_q.set_title("Optimal Excitation Trajectory")

        # position
        for d in range(self._fourier_traj.dof):
            _, linestyle = linestyles[d]
            plt_q.plot(x, q[:, d], label=(r"$q_"+str(d+1)+"$"), linestyle=linestyle)
        plt_q.legend()

        plt_q.set_ylabel(r'$q$ (rad or m)')

        # velocity
        plt_dq = fig.add_subplot(312)
        for d in range(self._fourier_traj.dof):
            _, linestyle = linestyles[d]
            plt_dq.plot(x, dq[:, d], label=(r"$\dot{q}_"+str(d+1)+"$"), linestyle=linestyle)

        plt_dq.legend()
        plt_dq.set_ylabel(r'$\dot{q}$ (rad/s or m/s)')

        # acceleration
        plt_ddq = fig.add_subplot(313)
        for d in range(self._fourier_traj.dof):
            #print('traj:', d)
            _, linestyle = linestyles[d]
            plt_ddq.plot(x, ddq[:, d], label=(r"$\ddot{q}_"+str(d+1)+"$"), linestyle=linestyle)

        plt_ddq.legend()
        plt_ddq.set_xlabel(r'$t$ (s)')
        plt_ddq.set_ylabel(r'$\ddot{q}$ (rad/s$^2$ or m/s$^2$)')
        #plt.tight_layout()
        plt.show()

    def plot_measured_traj(self):
        pass

    def plot_frame_traj(self, argu):
        x = self._fourier_traj.t
        map = ['x', 'y', 'z']

        const_size = len(self._const_frame_ind)

        subplotnum = const_size*100 + 11

        fig = plt.figure(2)
        for i in range(const_size):
            plt_q = fig.add_subplot(subplotnum + i)

            for d in range(3):
                _, linestyle = linestyles[d]
                plt_q.plot(x, self._frame_traj[i, :, d], label=(str(map[d])), linestyle=linestyle)

            plt_q.legend()
            plt_q.set_ylabel(r'$q$ (m)')
            plt_q.set_title('Frame ' + str(int(self._const_frame_ind[i]))+ ' Trajectory')
        plt.show()