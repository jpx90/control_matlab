tar_pos = ((sin(t) > 0) * 1 - 0.5) * 2;

tar_vel = zeros(max_n, 1) + 0;

tar_acc = zeros(max_n, 1) + 0;

tar_ang = ((mod((0 : max_n - 1)', 9000) < 3000) - 0.5) * 0.2;
% tar_ang = (t < 30) .* tar_ang + (t >= 30) .* cos(t - 30) * 0.1;
tar_ang = (t < 30) .* tar_ang ...
    + ((t >= 30 & t < 30 + pi) | (t > 30 + 3 * pi)) .* cos(t - 30) * 0.1 ...
    + (t >= 30 + pi & t <= 30 + 3 * pi) .* -0.1;

tar_ang_vel = zeros(max_n, 1) + 0;
% tar_ang_vel = ((mod((0 : max_n - 1)', 9000) < 3000) - 0.5) * 10;

tar_ang_acc = zeros(max_n, 1) + 0;
