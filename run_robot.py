#!/usr/bin/env python
from numpy import genfromtxt
import rospy
import csv
import numpy as np
import dvrk

modelname = 'test_psm_long'
testname = 'two'

name = './data/' + modelname + '/optimal_trajectory/' + testname
q = genfromtxt(name + '.csv', delimiter=',')

robotname = 'PSM1'

dof = len(q[0]) - 1
freq = q[0, -1]
a = q[:, 0:-1]

print(q[0, :],q.shape)
print(a[0, :], a.shape)

speedscale = 1
scale = 1

x = robotname[0:3] == 'PSM'
if x:
    p = dvrk.psm(robotname)
elif robotname[0:3] == 'MTM':
    p = dvrk.mtm(robotname)

r = rospy.Rate(freq * speedscale)
p.home()

# Home to start of trajectory based on CSV
if x and dof == 7:
    array = np.array([0,1,2,3,4,5])
    p.move_joint_some(a[0, 0:dof-1], array)
    p.move_jaw(q[0, -1])
else:
    array = np.linspace(0, dof, dof+1)
    p.move_joint_some(a[0, 0:dof], array)
print(array)

#states = np.zeros((len(q), 3 * dof))
states = np.zeros((len(q), 2 * dof))

print(states.shape)
i = 0
while i < len(a) and not rospy.is_shutdown():
    if x and dof ==7:
        p.move_joint_some(a[i, 0:dof-1], array, False)
        p.move_jaw(a[0, -1],False)

        # states[i][0:dof-1] = p.get_current_joint_position()[0:dof-1]
        # states[i][7] = p.get_current_jaw_position()
        # states[i][dof:dof * 2-1] = p.get_current_joint_velocity()[0:dof-1]
        # states[i][dof+7] = p.get_current_jaw_velocity()
        # states[i][dof * 2:dof * 3-1] = p.get_current_joint_effort()[0:dof-1]
        # #states[i][2 * dof + 7] = p.get_current_jaw_effort()
        # #print('it works')

        states[i][0:dof-1] = p.get_current_joint_position()[0:dof-1]
        states[i][7] = p.get_current_jaw_position()
        states[i][dof:dof * 2-1] = p.get_current_joint_effort()[0:dof-1]
        states[i][dof + 6] = p.get_current_jaw_effort()
        #print('it works')
    else:
        p.move_joint_some(q[i, :], array, False)

        # states[i][0:dof] = p.get_current_joint_position()[0:dof]
        # states[i][dof:dof*2] = p.get_current_joint_velocity()[0:dof]
        # states[i][dof*2:dof*3] = p.get_current_joint_effort()[0:dof]

        states[i][0:dof] = p.get_current_joint_position()[0:dof]
        states[i][dof:dof * 2] = p.get_current_joint_effort()[0:dof]

    r.sleep()
    i = i + 1

with open(name + '_results.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    for i in range(np.size(states, 0) - 1):
        wr.writerow(states[i])
