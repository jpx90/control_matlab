a = 1000;
b = 500;
max_n = a * 10 + b * 16 + 1000;
t = (1 : max_n)' * dt_s;

tick_mpc1 = a * 5;
tick_mpc2 = a * 7 + b * 8;

tar_pos = ((sin(t) > 0) * 1 - 0.5) * 2;

tar_vel = zeros(max_n, 1) + 0;

tar_acc = zeros(max_n, 1) + 0;

% tar_ang = zeros(max_n, 1) + 0;
tar_ang_max = 0.2;
tar_ang = [ ...
	zeros(a, 1);
	((mod((1 : a * 5)' - 1, a * 2) < a) - 0.5) * tar_ang_max * 2;
	cos((1 : b)' / b * pi) * tar_ang_max;
	zeros(a, 1) - tar_ang_max;
	cos((b + 1 : 8 * b)' / b * pi) * tar_ang_max;
	zeros(a, 1) + tar_ang_max;
	cos((1 : b)' / b * pi) * tar_ang_max;
	zeros(a, 1) - tar_ang_max;
	cos((b + 1 : 8 * b)' / b * pi) * tar_ang_max;
	zeros(a + 1000, 1) + tar_ang_max;];
clear r tar_ang_max

tar_ang_vel = zeros(max_n, 1) + 0;
% tar_ang_vel = ((mod((0 : max_n - 1)', 9000) < 3000) - 0.5) * 10;

tar_ang_acc = zeros(max_n, 1) + 0;
