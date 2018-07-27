import pandas as pd
from data_processing import load_trajectory_data, central_diff, central_2nd_diff, diff_and_filt_data, plot_trajectory_data
import copy
import scipy


# Load data
trajectory_data_file = '../data/trajectory/elbow_pitch_joint_states.csv'
trajectory_sampling_rate = 500
t, q_raw, tau_raw = load_trajectory_data(trajectory_data_file, trajectory_sampling_rate)
dof = q_raw.shape[1]
# Filter Cut-off Frequency Definition
fc_mult = 10.0
wf = 0.1
order = 6
fc = wf * order * fc_mult
print("cut frequency is {}".format(fc))

# Differentiation of raw position to get velocity and acceleration
# dq_raw = copy.deepcopy(q_raw)
# ddq_raw = copy.deepcopy(q_raw)
#
# for i in range(dof):
#     dq_raw[i] = central_diff(q_raw[i], 1.0/trajectory_sampling_rate)
#     ddq_raw[i] = central_2nd_diff(q_raw[i], 1.0/trajectory_sampling_rate)

q_f, dq_f, ddq_f, tau_f = diff_and_filt_data(dof, 1.0/trajectory_sampling_rate, q_raw, tau_raw, fc, fc, fc, fc)
plot_trajectory_data(t, q_raw, q_f, dq_f, ddq_f, tau_raw)

# Ordinary Least Square (OLS)
#xb_ols = scipy.linalg.lstsq(W, tau_f)