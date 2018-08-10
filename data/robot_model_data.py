from dynamics import Dynamics
from kinematics import Geometry
import robot_def
import sympy
import numpy

class RobotModel:
    def __init__(self, dyn):
        self.dof = dyn.rbt_def.dof
        self.coordinates = dyn.rbt_def.coordinates
        self.base_num = dyn.base_num
        self.base_param = dyn.base_param
        self.std_param = dyn.rbt_def.std_params
        self.bary_param = dyn.rbt_def.bary_params

        self.H_b_func = dyn.H_b_func
        self.H_func = dyn.H_func
        self.p_n_func = dyn.geom.p_n_func

        #Problems Loading matrix
        #self.T_0n = dyn.geom.T_0n[3] #Matrix?? 3 sucks

        self.T_0n = dyn.geom.T_0n[2]


