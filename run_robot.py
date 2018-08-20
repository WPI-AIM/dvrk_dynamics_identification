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

robotname = 'PSM2'

dof = len(q[0]) - 1
freq = q[0, -1]
a = q
print(q[0, :])

speedscale = 1
scale = 1

if robotname[0:3] == 'PSM':
    p = dvrk.psm(robotname)
elif robotname[0:3] == 'MTM':
    p = dvrk.mtm(robotname)

r = rospy.Rate(freq * speedscale)
p.home()

# Home to start of trajectory based on CSV
array = np.linspace(0, dof, dof+1)
print(array)

p.move_joint_some(q[0, :], array)
states = np.zeros((len(q), 3 * dof))

print(states.shape)
i = 0

while i < len(a) and not rospy.is_shutdown():
    p.move_joint_some(q[i, :], array, False)
    states[i][0:dof] = p.get_current_joint_position()[0:dof]
    states[i][dof:dof*2] = p.get_current_joint_velocity()[0:dof]
    states[i][dof*2:dof*3] = p.get_current_joint_effort()[0:dof]
    r.sleep()

    i = i + 1

with open(name + '_results.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    for i in range(np.size(states, 0) - 1):
        wr.writerow(states[i])
