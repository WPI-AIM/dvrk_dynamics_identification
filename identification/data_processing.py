# This file is originally adopted from https://github.com/cdsousa/wam7_dyn_ident
# and modified by Yan Wang
import math
import numpy as np
import scipy.signal
import pickle
import pandas as pd
import matplotlib.pyplot as plt
from utils import ml2r, Lmr2I, inertia_vec2tensor, inertia_tensor2vec
import sympy

# the format of file should be q0, tau0, q1, tau1, ..., qn, taun
def load_trajectory_data(file, freq):
    f = np.array(pd.read_csv(file, sep=',', header=None))
    row, col = f.shape
    sample_num = row
    dof = col/3


    print(type(f), f.shape)

    t = np.array(range(sample_num), dtype=float) / freq

    q = np.zeros((row, dof))
    dq = np.zeros((row, dof))
    tau = np.zeros((row, dof))

    for d in range(dof):
        q[:, d] = f[:, d]
        dq[:, d] = f[:, dof + d]
        tau[:, d] = f[:, 2*dof + d]

    return t, q, dq, tau


def central_diff(array, div, n=2):
    size = len(array)
    diff = np.zeros_like(array)
    if n == 1:
        diff[0] = (array[1] - array[0]) / div
        for i in range(1, size - 1):
            diff[i] = (array[i + 1] - array[i - 1]) / (2 * div)
        diff[size - 1] = (array[size - 1] - array[size - 2]) / div
    elif n == 2:
        diff[0] = (array[1] - array[0]) / div
        diff[1] = (array[2] - array[0]) / (2 * div)
        for i in range(2, size - 2):
            diff[i] = (- array[i + 2] + 8 * array[i + 1] - 8 * array[i - 1] + array[i - 2]) / (12 * div)
        diff[size - 2] = (array[size - 1] - array[size - 3]) / (2 * div)
        diff[size - 1] = (array[size - 1] - array[size - 2]) / div
    else:
        raise Exception('use n = 1 or 2')
    return diff


def central_2nd_diff(array, div, n=2):
    size = len(array)
    diff = np.zeros_like(array)
    if n == 1:
        diff[0] = (array[1 + 1] - 2 * array[1] + array[1 - 1]) / (div * div)
        for i in range(1, size - 1):
            diff[i] = (array[i + 1] - 2 * array[i] + array[i - 1]) / (div * div)
        diff[size - 1] = (array[(size - 2) + 1] - 2 * array[(size - 2)] + array[(size - 2) - 1]) / (div * div)
    elif n == 2:
        diff[0] = diff[1] = (array[1 + 1] - 2 * array[1] + array[1 - 1]) / (div * div)
        for i in range(2, size - 2):
            diff[i] = (- array[i + 2] + 16 * array[i + 1] - 30 * array[i] + 16 * array[i - 1] - array[i - 2]) / (
                        12 * div * div)
        diff[size - 1] = diff[size - 2] = (array[(size - 2) + 1] - 2 * array[(size - 2)] + array[(size - 2) - 1]) / (
                    div * div)
    else:
        raise Exception('use n = 1 or 2')
    return diff

#
# def filtfilt_simple(b, a, input_vector):
#     '''input_vector has shape (n,1)'''
#     forward = scipy.signal.lfilter(b, a, input_vector, axis=0)
#     return scipy.flipud(scipy.signal.lfilter(b, a, scipy.flipud(forward), axis=0))
#
#
# def lfilter_zi(b, a):
#     # compute the zi state from the filter parameters. see [Gust96].
#
#     # Based on:
#     # [Gust96] Fredrik Gustafsson, Determining the initial states in forward-backward
#     # filtering, IEEE Transactions on Signal Processing, pp. 988--992, April 1996,
#     # Volume 44, Issue 4
#
#     n = max(len(a), len(b))
#
#     zin = (numpy.eye(n - 1) - numpy.hstack((-a[1:n, numpy.newaxis],
#                                             numpy.vstack((numpy.eye(n - 2), numpy.zeros(n - 2))))))
#
#     zid = b[1:n] - a[1:n] * b[0]
#
#     zi_matrix = numpy.linalg.inv(zin) * (numpy.matrix(zid).transpose())
#     zi_return = []
#
#     # convert the result into a regular array (not a matrix)
#     for i in range(len(zi_matrix)):
#         zi_return.append(float(zi_matrix[i][0]))
#
#     return numpy.array(zi_return)
#
#
def filtfilt(b, a, x):
    # For now only accepting 1d arrays
    ntaps = max(len(a), len(b))
    edge = ntaps * 3

    if x.ndim != 1:
        raise ValueError("Filiflit is only accepting 1 dimension arrays.")

    # x must be bigger than edge
    if x.size < edge:
        raise ValueError("Input vector needs to be bigger than 3 * max(len(a),len(b).")

    if len(a) < ntaps:
        a = np.r_[a, np.zeros(len(b) - len(a))]

    if len(b) < ntaps:
        b = np.r_[b, np.zeros(len(a) - len(b))]

    zi = scipy.signal.lfilter_zi(b, a)

    # Grow the signal to have edges for stabilizing
    # the filter with inverted replicas of the signal
    s = np.r_[2 * x[0] - x[edge:1:-1], x, 2 * x[-1] - x[-1:-edge:-1]]
    # in the case of one go we only need one of the extrems
    # both are needed for filtfilt

    (y, zf) = scipy.signal.lfilter(b, a, s, -1, zi * s[0])

    (y, zf) = scipy.signal.lfilter(b, a, np.flipud(y), -1, zi * y[-1])

    return np.flipud(y[edge - 1:-edge + 1])
#
#
# def butter_lfilter(N, Wn, signal):
#     butter_b, butter_a = scipy.signal.butter(N, Wn)
#     filtered_signal = scipy.signal.lfilter(butter_b, butter_a, signal)
#     return filtered_signal
#
#
_inf = float('inf')


def butter_filtfilt(N, Wn, signal):
    if Wn == _inf: return signal[:]
    butter_b, butter_a = scipy.signal.butter(N, Wn)
    filtered_signal = filtfilt(butter_b, butter_a, signal)
    return filtered_signal
#
#

#
#
# def read_data(dof, h, rbtlogfile, trajreffile):
#     rbtlog = numpy.loadtxt(rbtlogfile)
#     s = rbtlog.shape[0]
#     t = numpy.array(rbtlog[:, 0])
#     q = numpy.array(rbtlog[:, 1:dof + 1])
#     tau = numpy.array(rbtlog[:, dof + 1:dof * 2 + 1])
#
#     h_avg = (t[-1] - t[0]) / len(t)
#     # print ('h avg',h_avg,'h nom',h)
#     if abs((h_avg / h) - 1) > 10e-5:
#         print('h nom != h avg')
#         print ('h avg', h_avg, 'h nom', h)
#
#     trajref = numpy.loadtxt(trajreffile)
#     reft = numpy.array([h * i for i in range(trajref.shape[0])])
#
#     return t, q, tau, reft, trajref
#
#
def diff_and_filt_data(dof, h, t, q_raw, dq_raw, tau_raw, fc_q, fc_tau, fc_dq, fc_ddq, cut_num = 200, filter_order=6):
    s = q_raw[0].shape[0]

    q = np.zeros_like(q_raw)
    dq = np.zeros_like(dq_raw)
    ddq = np.zeros_like(dq_raw)
    tau = np.zeros_like(tau_raw)

    if fc_q.shape[0] == 1:
        wc_q = [fc_q[0] * 2 * math.pi * h]*dof
        wc_dq = [fc_dq[0] * 2 * math.pi * h]*dof
        wc_ddq = [fc_ddq[0] * 2 * math.pi * h]*dof
        wc_tau = [fc_tau[0] * 2 * math.pi * h]*dof


    else:
        wc_q = fc_q * 2 * math.pi * h
        wc_dq = fc_dq * 2 * math.pi * h
        wc_ddq = fc_ddq * 2 * math.pi * h
        wc_tau = fc_tau * 2 * math.pi * h

    print('q_raw shape: {}'.format(q_raw.shape))
    for i in range(dof):
        q[:, i] = butter_filtfilt(filter_order, wc_q[i], q_raw[:, i])

        # joint_i_dq_raw = central_diff(q_raw[:, i], h, 2)
        # dq[:, i] = butter_filtfilt(filter_order, wc_dq, joint_i_dq_raw)
        dq[:, i] = butter_filtfilt(filter_order, wc_dq[i], dq_raw[:, i])

        # joint_i_ddq_raw = central_diff(joint_i_dq_raw, h, 2)
        joint_i_ddq_raw = central_diff(dq_raw[:, i], h, 2)
        ddq[:, i] = butter_filtfilt(filter_order, wc_ddq[i], joint_i_ddq_raw)
        # ddq[:,i] = central_diff( central_diff(q[:,i],h,2) ,h,2)

        tau[:, i] = butter_filtfilt(filter_order, wc_tau[i], tau_raw[:, i])
        # tau[:,i] = butter_lfilter( 3, wc_tau, tau_raw[:,i] )

    return t[cut_num:-cut_num],\
           q[cut_num:-cut_num, :], dq[cut_num:-cut_num, :], ddq[cut_num:-cut_num, :], tau[cut_num:-cut_num, :],\
           q_raw[cut_num:-cut_num, :], tau_raw[cut_num:-cut_num, :]


def plot_trajectory_data(t, q_raw, q_f, dq_f, ddq_f, tau_raw, tau_f):
    dof = q_raw.shape[1]
    plot_shape = 400 + dof*10
    print("plot shape: {}".format(plot_shape))

    fig = plt.figure()

    for i in range(dof):
        plt_q = fig.add_subplot(4, dof, i + 1)
        plt_q.plot(t, q_raw[:, i])
        plt_q.plot(t, q_f[:, i])
        if i == 0:
            plt_q.set_ylabel(r'$q$ (rad or m)')
        plt_q.set_title("Joint {}".format(i+1))

    for i in range(dof):
        plt_dq = fig.add_subplot(4, dof, i + 1 + dof)
        plt_dq.plot(t, dq_f[:, i])
        if i == 0:
            plt_dq.set_ylabel(r'$\dot{q}$ (rad/s or m/s)')

    for i in range(dof):
        plt_ddq = fig.add_subplot(4, dof, i + 1 + 2*dof)
        plt_ddq.plot(t, ddq_f[:, i])
        if i == 0:
            plt_ddq.set_ylabel(r'$\ddot{q}$ (rad/s$^2$ or m/s$^2$)')

    for i in range(dof):
        plt_tau = fig.add_subplot(4, dof, i + 1 + 3*dof)
        plt_tau.plot(t, tau_raw[:, i])
        plt_tau.plot(t, tau_f[:, i])
        plt_tau.set_xlabel(r'$t$ (s)')
        if i == 0:
            plt_tau.set_ylabel(r'$\tau$ (Nm) or $f$ (N)')

    plt.tight_layout()
    plt.show()


def plot_meas_pred_tau(t, tau_m, tau_p, joint_type, coordinates):
    sample_num, dof = tau_m.shape
    t = t - t[0]

    fig = plt.figure()

    font_size_def = 9.0

    for i in range(dof):
        plt_tau = fig.add_subplot(dof, 1, i + 1)
        plt_tau.margins(x=0.002, y=0.02)
        plt_tau.plot(t, tau_m[:, i], 'r', label="Measured", linewidth=1)
        plt_tau.plot(t, tau_p[:, i], 'b', label="Predicted", linewidth=1)
        plt_tau.plot(t, tau_p[:, i] - tau_m[:, i], 'k--', label="Error", linewidth=1)
        zeros = np.zeros(tau_p[:, i].shape)
        # plt_tau.plot(t, zeros, color='0.5', linewidth=0.75)
        if i == dof-1:
            plt_tau.set_xlabel(r'$t$ (s)', fontsize=font_size_def)
        if joint_type[i] == 'R':
            plt_tau.set_ylabel(r'$\tau^m_{}$ (Nm)'.format(coordinates[i].name[1:]), fontsize=font_size_def)
        else:
            plt_tau.set_ylabel(r'$f^m_{}$ (N)'.format(coordinates[i].name[1:], fontsize=font_size_def))
        # plt_tau.legend(['Measured', "Predicted"])
        if i == 0:
            plt_tau.legend(bbox_to_anchor=(0.0, 1.40, 1.0, .102), loc='upper center', ncol=3,
                           mode="expand", borderaxespad=0., fontsize=font_size_def)
        plt_tau.tick_params(labelsize=font_size_def)
    # plt.tight_layout()
    plt.show()


def gen_regressor(param_num, H, q, dq, ddq, tau):
    sample_num, dof = q.shape

    W = np.zeros((sample_num*dof, param_num))
    tau_s = np.zeros(sample_num*dof)

    for i in range(sample_num):
        vars_input = q[i, :].tolist() + dq[i, :].tolist() + ddq[i, :].tolist()
        W[i*dof:(i+1)*dof, :] = H(*vars_input)

        for d in range(dof):
            tau_s[i*dof + d] = tau[i, d]

    return W, tau_s


def barycentric2standard_params(x, rbt_def, Rs=None):
    i = 0
    i_link = 1
    x_out = []
    while i_link < rbt_def.frame_num:
        if rbt_def.use_inertia[i_link]:
            m = x[i + 9]
            rx, ry, rz = x[i + 6] / m, x[i + 7] / m, x[i + 8] / m
            r = [rx, ry, rz]
            L_mat = inertia_vec2tensor(x[i: i + 6])
            I_vec = inertia_tensor2vec(Lmr2I(L_mat, m, r))

            if Rs:
                R = Rs[i_link - 1]
                RI = np.matmul(R, np.matrix(inertia_vec2tensor(I_vec)).astype(np.float64))
                I_vec = inertia_tensor2vec(np.matmul(RI, R.transpose()))
                I_vec = np.array(I_vec).astype(np.float64).tolist()
                r = np.matmul(R, np.array(r).transpose()).tolist()[0]

            x_out += I_vec + r + [m]

            i += 10
        if rbt_def.use_friction[i_link]:
            if 'Coulomb' in rbt_def.friction_type:
                x_out += [x[i]]
                i += 1

            if 'viscous' in rbt_def.friction_type:
                x_out += [x[i]]
                i += 1

            if 'offset' in rbt_def.friction_type:
                x_out += [x[i]]
                i += 1

        if rbt_def.use_Ia[i_link]:
            x_out += [x[i]]
            i += 1

        if rbt_def.spring_dl[i_link] != None:
            x_out += [x[i]]
            i += 1

        i_link += 1

    return x_out


def params_array2table(param_value, rbt_def, std_or_bary):
    i = 0 # param_value count
    i_link = 1
    table = sympy.zeros(rbt_def.frame_num, 15+1)
    std_param_str = ['link', 'I_xx', 'I_xy', 'I_xz', 'I_yy', 'I_yz', 'I_zz', 'r_x', 'r_y', 'r_z', 'm', 'F_c', 'F_v', 'F_o', 'I_m',
                 'K']
    bary_param_str = ['link', 'L_xx', 'L_xy', 'L_xz', 'L_yy', 'L_yz', 'L_zz', 'l_x', 'l_y', 'l_z', 'm', 'F_c', 'F_v', 'F_o', 'I_m',
                 'K']
    param_str = std_param_str
    if std_or_bary == 'bary':
        param_str = bary_param_str

    n_sym = 0
    for k in range(len(param_str)):
        table[0, k] = param_str[k]

    while i_link < rbt_def.frame_num:
        table[i_link, 0] = i_link
        for j in range(10):
            if rbt_def.use_inertia[i_link]:
                table[i_link, j + 1] = param_value[i]
                i += 1
            else:
                table[i_link, j + 1] = n_sym

        if rbt_def.use_friction[i_link]:
            if 'Coulomb' in rbt_def.friction_type:
                table[i_link, 11] = param_value[i]
                i += 1
            else:
                table[i_link, 11] = n_sym

            if 'viscous' in rbt_def.friction_type:
                table[i_link, 12] = param_value[i]
                i += 1
            else:
                table[i_link, 12] = n_sym

            if 'offset' in rbt_def.friction_type:
                table[i_link, 13] = param_value[i]
                i += 1
            else:
                table[i_link, 13] = n_sym
        else:
            table[i_link, 11] = n_sym
            table[i_link, 12] = n_sym
            table[i_link, 13] = n_sym

        if rbt_def.use_Ia[i_link]:
            table[i_link, 14] += param_value[i]
            i += 1
        else:
            table[i_link, 14] = n_sym

        if rbt_def.spring_dl[i_link] != None:
            table[i_link, 15] += param_value[i]
            i += 1
        else:
            table[i_link, 15] = n_sym

        i_link += 1

    return table


def write_parameters2json(table, folder, name):
    json_data = {}
    for i in range(table.shape[0] - 1):
        link_num = i + 1
        link_data = {
            'I': [float(table[link_num, 1]),
                  float(table[link_num, 2]),
                  float(table[link_num, 3]),
                  float(table[link_num, 4]),
                  float(table[link_num, 5]),
                  float(table[link_num, 6])],
            'r': [float(table[link_num, 7]),
                  float(table[link_num, 8]),
                  float(table[link_num, 9])],
            'm': float(table[link_num, 10]),
            'Fc': float(table[link_num, 11]),
            'Fv': float(table[link_num, 12]),
            'Fo': float(table[link_num, 13]),
            'Im': float(table[link_num, 14]),
            'K': float(table[link_num, 15])
        }
        json_data[str(link_num)] = link_data

    import io, json, os, errno

    file_name = folder + name + '.json'

    if not os.path.exists(os.path.dirname(file_name)):
        try:
            os.makedirs(os.path.dirname(file_name))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    with io.open(file_name, 'wb') as f:
        f.write(json.dumps(json_data, sort_keys=True, indent=4))

    print("Parameters have been written into [{}] successfully!".format(file_name))


def trans_inertia(I, r, R):
    I1 = np.matmul(np.matmul(R.transpose(), I), R)
    r1 = np.matmul(R.transpose(), r)
    return I1, r1