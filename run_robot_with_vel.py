
#!/usr/bin/env python
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
#modelname = 'mtm_2spring_tendon'
#model_name = 'mtm'

model_name = 'psm_simplified'
#model_name = 'psm_simple_coupled'
# testname = 'one'

#testname = 'one'

robotname = 'PSM1'
#robotname = 'MTMR'


motor2dvrk_psm = np.array([[1.0186, 0, 0], [-.8306, .6089, .6089], [0, -1.2177, 1.2177]])
motor2dvrk_mtm = np.array([[1.0, 0, 0], [-1.0, 1.0, 0], [0.6697, -0.6697, 1.0]])
#scales = np.array([0.8, 0.8, 0.8, 1, 1, 1, 1])
#scales = np.array([0.8, 0.8, 0.8, 1, 1, 1])

#PSM
scales = np.array([1, 1, 1, 1, 1, 1, 1])
#scales = np.array([0.5, 0.5, 0.5, 0.75, 0.75, 0.75, 0.75])
#MTM
#scales = np.array([0.7, 0.85, 0.85, 0.85, 1, 1, 1])

# wait for a short period of time before recording data
stable_time = 5
sampling_time = 30
sampling_rate = 500
speedscale = 1


trajectory_name = 'one'
testname = trajectory_name

model_folder = 'data/' + model_name + '/model/'
robot_model = load_data(model_folder,model_name)


trajectory_folder = 'data/' + model_name +'/optimal_trajectory/'
dof, fourier_order, base_freq, traj_optimizer_result, reg_norm_mat = load_data(trajectory_folder, trajectory_name)
print (traj_optimizer_result.shape)
print (dof)


if scales.shape[0] != dof:
	raise Excitation()

# Scale it first 
param_num_for_one_joint = 1+2*fourier_order
for i in range(dof):
	traj_optimizer_result[1+i*param_num_for_one_joint: 7+i*param_num_for_one_joint] * scales[i]

# Generate trajectory data with ramp-up
x = FourierTraj(dof, fourier_order, base_freq,
	frequency=sampling_rate, stable_time=stable_time, final_time=sampling_time)
q, _, _ = x.fourier_base_x2q(traj_optimizer_result)
# plt.plot(q[:,0])
# plt.show()
print (q.shape)

a = q

print("data shape: {}".format(a.shape))

print(dof)

b = copy.deepcopy(a)


is_psm = robotname[0:3] == 'PSM'
if is_psm:
	p = dvrk.psm(robotname)
	for i in range(a.shape[0]):
		b[i, 4] = 1.0186 * a[i, 4]
		b[i, 5] = -0.8306 * a[i, 4] + 0.6089 * a[i, 5] + 0.6089 * a[i, 6]
		b[i, 6] = -1.2177 * a[i, 5] +1.2177 * a[i, 6]

elif robotname[0:3] == 'MTM':
	p = dvrk.mtm(robotname)
	for i in range(a.shape[0]):
		b[i, 2] = -a[i, 1] + a[i, 2]
		b[i, 3] = 0.6697 * a[i, 1] - 0.6697 * a[i, 2] + a[i, 3]



a = b * scales


p.home()

rospy.sleep(3)

# Home to start of trajectory based on CSV
if is_psm and dof == 7:
	jonits_array = np.array([0,1,2,3,4,5])
	print(a[0,:])
	p.move_joint_some(a[0, 0:dof-1], jonits_array)
	p.move_jaw(a[0, -1])
else:
	jonits_array = np.array([d for d in range(dof)])
	p.move_joint_some(a[0, :], jonits_array)
print("jonits_array: {}".format(jonits_array))

#states = np.zeros((len(q), 3 * dof))
start_cnt = int(sampling_rate*stable_time)

max_state_num = len(q) - start_cnt
states = np.zeros((max_state_num, 3 * dof))

print("states shape: {}".format(states.shape))
rospy.sleep(3)

r = rospy.Rate(sampling_rate * speedscale)
# Excitation
i = 0
state_cnt = 0
while i < len(a) and not rospy.is_shutdown():
	if is_psm and dof ==7:
		#print(a[i,-1])
		p.move_joint_some(a[i, 0:dof-1], jonits_array, interpolate=False,blocking=False)
		p.move_jaw(a[i, -1],interpolate=False,blocking=False)

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

			#print('error', a[i,:] - states[state_cnt][0:dof])
			
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

			states[state_cnt][dof*2:dof * 3] = p.get_current_joint_effort()[0:dof]


	r.sleep()
	i = i + 1

#Coupling revert
motor_state = copy.deepcopy(states)
if is_psm:
	for i in range(states.shape[0]):
		motor_state[i, 4:7] = np.matmul(np.linalg.inv(motor2dvrk_psm), states[i, 4:7])
		motor_state[i, 11:14] = np.matmul(np.linalg.inv(motor2dvrk_psm), states[i, 11:14])
		motor_state[i, 18:22] = np.matmul(motor2dvrk_psm.transpose(), states[i, 18:22])
	states = motor_state
else:
	for i in range(states.shape[0]):
		motor_state[i, 1:4] = np.matmul(np.linalg.inv(motor2dvrk_mtm), states[i, 1:4])
		motor_state[i, 8:11] = np.matmul(np.linalg.inv(motor2dvrk_mtm), states[i, 8:11])
		motor_state[i, 15:18] = np.matmul(motor2dvrk_mtm.transpose(), states[i, 15:18])
	states = motor_state
	
# Save data
data_file_dir = './data/' + model_name + '/measured_trajectory/' + testname + '_results.csv'

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
