import dill as pickle
import sympy
import json
import os.path
import numpy as np
from robot_def import RobotDef
from kinematics import Geometry
from dynamics import Dynamics
from trajectory_optimization import TrajOptimizer
from trajectory_optimization import TrajPlotter
import time
from utils import new_sym


model_folder = 'data/model/'
trajectory_folder = 'data/trajectory/'

two_link_robot_model_file = model_folder + 'two_link_robot_model.pkl'


# Create joint variables and define their relations
q0, q1, q2, q3, q4, q5, q6, q7, q8, q9 = new_sym('q:10')
# q3 = -q2 + q8
# q9 = -q8 + q2

robot_def = RobotDef([(0,   -1, [1],    0, 0, 0, 0),
                      (1,   0,  [2],    0, 0, -0.21537, q1),
                      (2,   1,  [3],     0, -sympy.pi/2, 0, q2+sympy.pi/2)],
                     dh_convention='mdh',
                     friction_type=['Coulomb', 'viscous', 'offset'])

geom = Geometry(robot_def)

dyn = Dynamics(robot_def, geom)


#if not os.path.exists(two_link_robot_model_file):
with open(two_link_robot_model_file, 'wr') as f:
    pickle.dump(dyn.H_b, f)

H_b = None
if os.path.exists(two_link_robot_model_file):
    with open(two_link_robot_model_file, 'rb') as f:
        H_b = pickle.load(f)

print(H_b - dyn.H_b)
# import io, json
# with io.open('two_link_robot_model_file', 'w', encoding='utf-8') as f:
#   f.write(json.dumps(data, ensure_ascii=False))