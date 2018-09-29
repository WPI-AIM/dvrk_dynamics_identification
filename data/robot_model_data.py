from dynamics import Dynamics
from kinematics import Geometry
import robot_def
import sympy
import numpy

class RobotModel:
    def __init__(self, dyn):
        self.dof = dyn.rbt_def.dof
        self.coordinates = dyn.rbt_def.coordinates
        self.coordinates_joint_type = dyn.rbt_def.coordinates_joint_type
        self.base_num = dyn.base_num
        self.base_param = dyn.base_param
        self.std_param = dyn.rbt_def.std_params
        self.bary_param = dyn.rbt_def.bary_params

        #self.H_b = dyn.H_b
        #self.H = dyn.H

        self.H_b_func = dyn.H_b_func
        self.H_func = dyn.H_func
        self.p_n_func = dyn.geom.p_n_func

        self.frame_num = dyn.rbt_def.frame_num
        self.use_inertia = dyn.rbt_def.use_inertia
        self.use_friction = dyn.rbt_def.use_friction
        self.friction_type = dyn.rbt_def.friction_type
        self.use_Ia = dyn.rbt_def.use_Ia
        self.spring_num = dyn.rbt_def.spring_num

        #Problems Loading matrix
        #self.T_0n = dyn.geom.T_0n[3] #Matrix?? 3 sucks
        #self.T_0n = dyn.geom.T_0n


