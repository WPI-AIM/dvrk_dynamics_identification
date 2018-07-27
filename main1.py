import numpy as np
import sympy
from robot_def import RobotDef
from kinematics import Geometry
from dynamics import Dynamics
from trajectory_optimization import TrajOptimizer
from trajectory_optimization import TrajPlotter
from utils import new_sym
import time


# Create joint variables and define their relations
q0, q1, q2, q3, q4, q5, q6, q7, q8, q9 = new_sym('q:10')
# q3 = -q2 + q8
# q9 = -q8 + q2

start_time = time.time()
# frame number, previous frame number, next frame numbers, a, alpha, d, theta
# robot_def = RobotDef([(0,   -1, [1],    0, 0, 0, 0),
#                       (1,   0,  [2, 8], 0, 0, -0.21537, q1),
#                       (2,   1,  [3],    0, -sympy.pi/2, 0, q2+sympy.pi/2),
#                       (3,   2,  [4],    0.279, 0, 0, q3+sympy.pi/2),
#                       (4,   3,  [5],    0.365, -sympy.pi/2, 0.151, q4),
#                       (5,   4,  [6],    0, sympy.pi/2, 0, q5),
#                       (6,   5,  [7],    0, -sympy.pi/2, 0, q6+sympy.pi/2),
#                       (7,   6,  [],     0, -sympy.pi/2, 0, q7),
#                       (8,   1,  [9],    0, -sympy.pi/2, 0, q8+sympy.pi),
#                       (9,   8,  [],     0.1, 0, 0, q9-sympy.pi/2)],
#                      dh_convention='mdh',
#                      friction_type=['Coulomb', 'viscous', 'offset'])


robot_def = RobotDef([(0,   -1, [1],    0,      0,              0,          0),
                      (1,   0,  [2],    0,      0,              -0.21537,   q1),
                      (2,   1,  [3],    0,      -sympy.pi/2,    0,          q2+sympy.pi/2),
                      (3,   2,  [4],    0.279,  0,              0,          q3 + sympy.pi / 2)],
                     dh_convention='mdh',
                     friction_type=['Coulomb', 'viscous', 'offset'])

geom = Geometry(robot_def)
geom.draw_geom()

dyn = Dynamics(robot_def, geom)

traj_optimizer = TrajOptimizer(dyn, 6, 0.1,
                               joint_constraints=[(q1, -np.pi/2, np.pi/2, -2*np.pi, 2*np.pi),
                                                  (q2, -np.pi/2, np.pi/2, -2*np.pi, 2*np.pi)])
traj_optimizer.optimize()

traj_plotter = TrajPlotter(traj_optimizer.fourier_traj)
traj_plotter.plot_desired_traj(traj_optimizer.x_result)
print(time.time() - start_time)
