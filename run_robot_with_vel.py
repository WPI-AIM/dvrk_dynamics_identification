#!/usr/bin/env python
from numpy import genfromtxt
import rospy
import csv
import numpy as np
import dvrk
import os.path
import os
import errno


# Fist, several things we have to define before running it
# modelname = 'test_psm_long'
modelname = 'mtm'
# modelname = 'mtm_3links_parallel'
# modelname = 'mtm_4links_parallel'

# testname = 'one'
# testname = 'two'
testname = 'two_normalized'

# robotname = 'PSM1'
robotname = 'MTMR'

speedscale = 0.92
scale = 0.75
#scales = np.array([0.8, 0.8, 0.8, 1, 1, 1, 1])
scales = np.array([0.60, 0.90, 0.90, 1, 1, 1, 1])

# wait for a short period of time before recording data
stable_time = 5


name = './data/' + modelname + '/optimal_trajectory/' + testname
q = genfromtxt(name + '.csv', delimiter=',')


dof = len(q[0]) - 1
freq = q[0, -1]
# a = q[:, 0:-1] * scale
a = q[:, 0:-1] * scales

# print(q[0, :],q.shape)
# print(a[0, :], a.shape)
print("data shape: {}".format(a.shape))


is_psm = robotname[0:3] == 'PSM'
if is_psm:
    p = dvrk.psm(robotname)
elif robotname[0:3] == 'MTM':
    p = dvrk.mtm(robotname)

# deal with the parallelogram, where q3 = q8 - q2
if not is_psm:
    if dof == 7 or dof == 3 or dof == 4:
        a[:, 2] = a[:, 2] - a[:, 1]


p.home()

rospy.sleep(3)

# Home to start of trajectory based on CSV
if is_psm and dof == 7:
    jonits_array = np.array([0,1,2,3,4,5])
    p.move_joint_some(a[0, 0:dof-1], jonits_array)
    p.move_jaw(q[0, -1])
else:
    jonits_array = np.array([d for d in range(dof)])
    p.move_joint_some(a[0, :], jonits_array)
print("jonits_array: {}".format(jonits_array))

#states = np.zeros((len(q), 3 * dof))
start_cnt = int(freq*stable_time)
max_state_num = len(q) - start_cnt
states = np.zeros((max_state_num, 3 * dof))

print("states shape: {}".format(states.shape))
rospy.sleep(3)

r = rospy.Rate(freq * speedscale)
# Excitation
i = 0
state_cnt = 0
while i < len(a) and not rospy.is_shutdown():
    if is_psm and dof ==7:
        p.move_joint_some(a[i, 0:dof-1], jonits_array, False)
        p.move_jaw(a[0, -1],False)

        # states[i][0:dof-1] = p.get_current_joint_position()[0:dof-1]
        # states[i][7] = p.get_current_jaw_position()
        # states[i][dof:dof * 2-1] = p.get_current_joint_velocity()[0:dof-1]
        # states[i][dof+7] = p.get_current_jaw_velocity()
        # states[i][dof * 2:dof * 3-1] = p.get_current_joint_effort()[0:dof-1]
        # #states[i][2 * dof + 7] = p.get_current_jaw_effort()
        # #print('it works')
        if i >= start_cnt:
            state_cnt = i - start_cnt

            states[state_cnt][0:dof - 1] = p.get_current_joint_position()[0:dof-1]
            states[state_cnt][dof-1] = p.get_current_jaw_position()
            
            states[state_cnt][dof:dof*2 - 1] = p.get_current_joint_velocity()[0:dof-1]
            states[state_cnt][dof*2 - 1] = p.get_current_jaw_velocity()

            states[state_cnt][dof*2:dof * 3 - 1] = p.get_current_joint_effort()[0:dof-1]
            states[state_cnt][dof*3 - 1] = p.get_current_jaw_effort()
        #print('it works')

    else:
        p.move_joint_some(a[i, :], jonits_array, False)

        # states[i][0:dof] = p.get_current_joint_position()[0:dof]
        # states[i][dof:dof*2] = p.get_current_joint_velocity()[0:dof]
        # states[i][dof*2:dof*3] = p.get_current_joint_effort()[0:dof]
        if i >= start_cnt:
            state_cnt = i - start_cnt
            states[state_cnt][0:dof] = p.get_current_joint_position()[0:dof]
            states[state_cnt][dof:dof*2] = p.get_current_joint_velocity()[0:dof]

            if dof == 7 or dof == 3 or dof == 4:
                states[state_cnt][2] = states[state_cnt][1] + states[state_cnt][2]
                states[state_cnt][dof + 2] = states[state_cnt][dof + 1] + states[state_cnt][dof + 2]
            states[state_cnt][dof*2:dof * 3] = p.get_current_joint_effort()[0:dof]

    r.sleep()
    i = i + 1


# Save data
data_file_dir = './data/' + modelname + '/measured_trajectory/' + testname + '_results.csv'

if not os.path.exists(os.path.dirname(data_file_dir)):
    try:
        os.makedirs(os.path.dirname(data_file_dir))
    except OSError as exc:  # Guard against race condition
        if exc.errno != errno.EEXIST:
            raise

with open(data_file_dir, 'w+') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    for i in range(np.size(states, 0) - 1):
        wr.writerow(states[i])
