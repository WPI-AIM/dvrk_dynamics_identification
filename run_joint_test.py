# !/usr/bin/env python
from numpy import genfromtxt
import rospy
import csv
import numpy as np
import dvrk
import os.path
import os
import errno
import copy
from utils import load_data
from trajectory_optimization import FourierTraj
import matplotlib.pyplot as plt

# Fist, several things we have to define before running it

model_name = 'mtm'
# model_name = 'psm_simplified'


# robotname = 'PSM1'
robotname = 'MTMR'

test_name = 'one'

motor2dvrk_psm = np.array([[1.0186, 0, 0], [-.8306, .6089, .6089], [0, -1.2177, 1.2177]])
motor2dvrk_mtm = np.array([[1.0, 0, 0], [-1.0, 1.0, 0], [0.6697, -0.6697, 1.0]])


# wait for a short period of time before recording data
stable_time = 5
sampling_time = 30
sampling_rate = 200
speed = 0.2

model_folder = 'data/' + model_name + '/model/'
robot_model = load_data(model_folder, model_name)

trajectory_folder = 'data/' + model_name + '/optimal_trajectory/'

p.home()

rospy.sleep(3)

jonits_array = np.array([d for d in range(7)])


q_start = -0.5
q_end = 0.5

q_trajectory = np.linspace(q_start, q_end, num=int(sampling_rate * (q_end - q_start) / speed))
states = np.zeros((q_trajectory.shape[0]*2, 3))

q = np.array([0, 0, 0, q_start, 0, 0, 0])
p.move_joint_some(q, jonits_array)

joint_num = 3

state_cnt = 0

r = rospy.Rate(sampling_rate)

if not rospy.is_shutdown():
    for i in range(q_trajectory.shape[0]):
        p.move_joint_some([q_trajectory[i]], [3], interpolate=False, blocking=False)

        states[state_cnt][0] = p.get_current_joint_position()[joint_num]
        states[state_cnt][1] = p.get_current_joint_velocity()[joint_num]
        states[state_cnt][2] = p.get_current_joint_effort()[joint_num]
        state_cnt += 1

        r.sleep()

    for i in range(q_trajectory.shape[0]):
        p.move_joint_some([q_trajectory[q_trajectory.shape[0] - i]], [3], interpolate=False, blocking=False)

        states[state_cnt][0] = p.get_current_joint_position()[joint_num]
        states[state_cnt][1] = p.get_current_joint_velocity()[joint_num]
        states[state_cnt][2] = p.get_current_joint_effort()[joint_num]
        state_cnt += 1

        r.sleep()




# Save data
data_file_dir = './data/' + model_name + '/joint_test/' + test_name + '_results.csv'

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
