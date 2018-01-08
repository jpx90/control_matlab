function loop_simulation

addpath('../lib');

max_n = 30000;
dt_s = 0.001;
t = (1 : max_n)' * dt_s;

GRAVITY = 9.81;

pos = zeros(max_n, 1);
vel = zeros(max_n, 1);
acc = zeros(max_n, 1);
ang = zeros(max_n, 1);
ang_vel = zeros(max_n, 1);
ang_acc = zeros(max_n, 1);
act = zeros(max_n, 1);

tar_pos = zeros(max_n, 1) + 10;
tar_vel = zeros(max_n, 1) + 0;
tar_acc = zeros(max_n, 1) + 0;
tar_ang = zeros(max_n, 1) + 0;
tar_ang_vel = zeros(max_n, 1) + 0;
tar_ang_acc = zeros(max_n, 1) + 0;

%% Model

act_to_acc_iir_index = 0.06;
act_to_acc_delay_n = 20;

%% Loop

for i = 1000 : max_n
	ang_acc(i) = act(i - act_to_acc_delay_n) * act_to_acc_iir_index + ang_acc(i - 1) * (1 - act_to_acc_iir_index);
	ang_vel(i) = ang_vel(i - 1) + ang_acc(i) * dt_s;
	ang(i) = ang(i - 1) + ang_vel(i) * dt_s;
	acc(i) = tan(ang(i)) * GRAVITY;
	vel(i) = vel(i - 1) + acc(i) * dt_s;
	pos(i) = pos(i - 1) + vel(i) * dt_s;
	
	tar_vel(i) = abs_constrain((tar_pos(i) - pos(i)) * 2, 5);
	tar_acc(i) = abs_constrain((tar_vel(i) - vel(i)) * 5, 5);
	tar_ang(i) = atan(tar_acc(i) / GRAVITY);
	tar_ang_vel(i) = abs_constrain((tar_ang(i) - ang(i)) * 5, 5);
	tar_ang_acc(i) = abs_constrain((tar_ang_vel(i) - ang_vel(i)) * 12, 100);
	act(i) = tar_ang_acc(i);
end;

figure(1);
ax(1) = subplot(3, 2, 1); plot(t, [tar_pos, pos]); grid on; legend('tar pos', 'pos');
ax(2) = subplot(3, 2, 3); plot(t, [tar_vel, vel]); grid on; legend('tar vel', 'vel');
ax(3) = subplot(3, 2, 5); plot(t, [tar_acc, acc]); grid on; legend('tar acc', 'acc');
ax(4) = subplot(3, 2, 2); plot(t, [tar_ang, ang]); grid on; legend('tar ang', 'ang');
ax(5) = subplot(3, 2, 4); plot(t, [tar_ang_vel, ang_vel]); grid on; legend('tar ang vel', 'ang vel');
ax(6) = subplot(3, 2, 6); plot(t, [tar_ang_acc, ang_acc]); grid on; legend('tar ang acc', 'ang acc');

linkaxes(ax, 'x');

end