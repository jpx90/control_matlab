function ang_vel_ctrl_test(model_iir_index, model_delay_ms, index, kp_union, n_max)

dt = 0.001;
N = size(kp_union, 1);

t = (0 : n_max)' * dt;
ang_vel = zeros(N, n_max + 1);
ang_acc = zeros(N, n_max + 1);
act = zeros(N, n_max + 1);

sim_resp = zeros(N, 1);
iir_omega = zeros(N, 1);

for i = 1000 : n_max + 1
	ang_acc(:, i) = ang_acc(:, i - 1) * (1 - model_iir_index) ...
		+ act(:, i - model_delay_ms - 1) * model_iir_index;
	ang_vel(:, i) = ang_vel(:, i - 1) + ang_acc(:, i - 1) * dt;
	
	iir_omega = iir_omega * 0.9 + ang_vel(:, i) * 0.1;
	sim_resp = sim_resp * (1 - index) + act(:, i - 1) * index;
	
	temp = (1 - iir_omega) .* kp_union;
% 	act(:, i) = temp;
	act(:, i) = (1.5 * temp - sim_resp) * 2;
end

max_x = zeros(N, 1);
for j = 1 : N
	max_x(j) = max(ang_vel(j, :));
end

subplot(1, 2, 1);
p = (ang_vel > 0.9) .* ang_vel + (ang_vel <= 0.9) .* 0.9;
pcolor(t, kp_union, p);
shading interp;
colorbar;
% caxis([0.9, 1.2]);
xlabel('t / sec');
ylabel('kp');
title(['Model: ', num2str(model_iir_index), ' ', ...
	num2str(model_delay_ms), ' ms index: ', num2str(index)]);

subplot(1, 2, 2);
plot(kp_union, max_x);
grid on;
xlabel('kp');

end
