from dh_def import *
from kinematics import *

# Create joint variables and define their relations
q0, q1, q2, q3, q4, q5, q6, q7, q8, q9 = new_sym('q:10')
q3 = -q2 + q8
q9 = -q8 + q2

#DH definition in the order of joint_type, a, alpha, d, theta, dh_type, prev, succ
dh0 = DHDef(0, 'R', 0, 0, 0, 0, 'mdh', None, [])
dh1 = DHDef(1, 'R', 0, 0, -0.21537, q1, 'mdh', dh0, [])
dh2 = DHDef(2, 'R', 0, -sympy.pi/2, 0, q2+sympy.pi/2, 'mdh', dh1, [])
dh3 = DHDef(3, 'R', 0.279, 0, 0, q3+sympy.pi/2, 'mdh', dh2, [])
dh4 = DHDef(4, 'R', 0.365, -sympy.pi/2, 0.151, q4, 'mdh', dh3, [])
dh5 = DHDef(5, 'R', 0, sympy.pi/2, 0, q5, 'mdh', dh4, [])
dh6 = DHDef(6, 'R', 0, -sympy.pi/2, 0, q6+sympy.pi/2, 'mdh', dh5, [])
dh7 = DHDef(7, 'R', 0, -sympy.pi/2, 0, q7, 'mdh', dh6, [])

dh8 = DHDef(8, 'R', 0, -sympy.pi/2, 0, q8+sympy.pi, 'mdh', dh1, [])
dh9 = DHDef(9, 'R', 0.1, 0, 0, q9-sympy.pi/2, 'mdh', dh8, [])

dh0.set_succ([dh1])
dh1.set_succ([dh2, dh8])
dh2.set_succ([dh3])
dh3.set_succ([dh4])
dh4.set_succ([dh5])
dh5.set_succ([dh6])
dh6.set_succ([dh7])
dh8.set_succ([dh9])

kin = Kinematics(dh0)
kin.cal_transfmats()
kin.draw_frames()
