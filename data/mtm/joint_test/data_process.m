data = csvread('/home/yan/code/dyn_ident_py/data/mtm/joint_test/one_results.csv');
data_size = size(data);
data_size(1) = round(data_size(1)/2) + 1;
data_remove = 100;
data_start1 = data_remove;
data_end1 =  data_size(1) - data_remove;

data_start2 = data_size(1) + data_remove;
data_end2 =  2*data_size(1) - data_remove;

pos_clcok = data(data_start1:data_end1, 1);
tau_clock = data(data_start1:data_end1, 3);

pos_cc = data(data_start2:data_end2, 1);
tau_cc = data(data_start2:data_end2, 3);

plot(data(data_start1:data_end1, 1), data(data_start1:data_end1, 3), '.')
hold on
plot(data(data_start2:data_end2, 1), data(data_start2:data_end2, 3),'.')

pos = data(data_start1:data_end1, 1);
%tau_middle = (f_clock(pos) + f_cc(pos)).*0.5;
tau_middle = f_s(pos);
plot(pos, tau_middle, '-', 'LineWidth', 2)

plot(data(data_start1:data_end1, 1), f_clock(data(data_start1:data_end1, 1)), '-','LineWidth', 1.5)

plot(data(data_start2:data_end2, 1), f_cc(data(data_start2:data_end2, 1)), '-', 'LineWidth', 1.5)



xlabel('q_4 (rad)')
ylabel('\tau_4 (Nm)')
legend('\tau_{+}', '\tau_{-}', '\tau_{c}', '\tau_{+} fitting', '\tau_{-} fitting')