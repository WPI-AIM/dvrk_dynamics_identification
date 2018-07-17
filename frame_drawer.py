import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import axes3d
from itertools import product, combinations



class FrameDrawer:
    def __init__(self):
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(111, projection='3d', aspect='equal')
        plot_range = 1.5
        self._ax.set_xlim(-0.2, plot_range)
        self._ax.set_ylim(-plot_range*0.5, plot_range*0.5)
        self._ax.set_zlim()

    def draw_frame(self, F):
        """Draws a coordinate reference frame from a 4x4 matrix.
        Adds an extra thick line along the Z axis and around the
        origin to simulate the joint in a robot segment"""

        R = F[0:3, 0:3]  # extract Rot matrix
        T = F[0:3, 3]  # extract traslation matrix
        p1 = np.matrix([[0.5], [0], [0]])  # get the 3 other points of the frame
        p2 = np.matrix([[0], [0.5], [0]])
        p3 = np.matrix([[0], [0], [0.5]])
        p1 = R * p1  # Rotate the points
        p2 = R * p2
        p3 = R * p3
        x = T.item(0)  # Get the origin's position of the frame
        y = T.item(1)
        z = T.item(2)
        # draw the line from the origin to each of the points
        self._ax.plot([x, x + p1.item(0)], [y, y + p1.item(1)], [z, z + p1.item(2)])
        self._ax.plot([x, x + p2.item(0)], [y, y + p2.item(1)], [z, z + p2.item(2)])
        self._ax.plot([x, x + p3.item(0)], [y, y + p3.item(1)], [z, z + p3.item(2)])
        l = 0.1  # draw the extra thicker line that simulates the
        # robot's joint
        self._ax.plot([x - p3.item(0) * l, x + p3.item(0) * l],
                        [y - p3.item(1) * l, y + p3.item(1) * l],
                        [z - p3.item(2) * l, z + p3.item(2) * l],
                        linewidth=6, color="k")

    def drawSegment(self, F1, F2):
        """Draws a thick segment between two Frames
        given as arguments."""
        T1 = F1[0:3, 3]  # traslation matrix 1
        T2 = F2[0:3, 3]  # traslation matrix 2

        self._ax.plot([T1.item(0), T2.item(0)],
                        [T1.item(1), T2.item(1)],
                        [T1.item(2), T2.item(2)],
                        linewidth=3, color="k")

    def show(self):
        plt.show()