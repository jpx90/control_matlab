tar_pos = ((sin(t) > 0) * 1 - 0.5) * 2;

tar_vel = zeros(max_n, 1) + 0;

tar_acc = zeros(max_n, 1) + 0;

% tar_ang = zeros(max_n, 1) + 0;
tar_ang_max = 0.5;
tar_ang = ((mod((0 : max_n - 1)', 9000) < 3000) - 0.5) * tar_ang_max * 2;
r = 3;
tar_ang = (t < 30) .* tar_ang ...
    + ((t >= 30 & t < 30 + pi / r) | (t > 30 + 3 * pi / r)) .* cos((t - 30) * r) * tar_ang_max ...
    + (t >= 30 + pi / r & t <= 30 + 3 * pi / r) .* -tar_ang_max;
tar_ang(30000 : 31999) = zeros(2000, 1) - 0.5;
clear r tar_ang_max

tar_ang_vel = zeros(max_n, 1) + 0;
% tar_ang_vel = ((mod((0 : max_n - 1)', 9000) < 3000) - 0.5) * 10;

tar_ang_acc = zeros(max_n, 1) + 0;
