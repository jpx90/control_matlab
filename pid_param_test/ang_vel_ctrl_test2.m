function ang_vel_ctrl_test2(model_iir_index, model_delay_ms, index_union, kp, n_max)

dt = 0.001;
N = size(index_union, 1);

t = (0 : n_max)' * dt;
ang_vel = zeros(N, n_max + 1);
ang_acc = zeros(N, n_max + 1);
act = zeros(N, n_max + 1);

sim_resp = zeros(N, 1);
iir_omega = zeros(N, 1);

max_n = zeros(N, 1);
max_x = zeros(N, 1);
min_n = zeros(N, 1);
min_x = zeros(N, 1);

for i = 1000 : n_max + 1
	ang_acc(:, i) = ang_acc(:, i - 1) * (1 - model_iir_index) ...
		+ act(:, i - model_delay_ms - 1) * model_iir_index;
	ang_vel(:, i) = ang_vel(:, i - 1) + ang_acc(:, i - 1) * dt;
	
	iir_omega = iir_omega * 0.9 + ang_vel(:, i) * 0.1;
	sim_resp = sim_resp .* (1 - index_union) + act(:, i - 1) .* index_union;
	
	temp = (1 - iir_omega) * kp;
% 	act(:, i) = temp;
	act(:, i) = (1.5 * temp - sim_resp) * 2;
	
	for j = 1 : N
		if max_n(j) == 0
			if ang_vel(j, i) < ang_vel(j, i - 1)
				max_n(j) = i - 1;
				max_x(j) = ang_vel(j, i - 1);
			end
		elseif min_n(j) == 0
			if ang_vel(j, i) > ang_vel(j, i - 1)
				min_n(j) = i - 1;
				min_x(j) = ang_vel(j, i - 1);
			end
		end
	end
end

p = ang_vel;
for j = 1 : N
	for i = 1 : max_n(j)
		p(j, i) = max_x(j);
	end
end

subplot(1, 2, 1);
pcolor(t, index_union, p);
shading interp;
colorbar;
% caxis([0.9, 1.2]);
xlabel('t / sec');
ylabel('index');
title(['Model: ', num2str(model_iir_index), ' ', ...
	num2str(model_delay_ms), ' ms kp: ', num2str(kp)]);

subplot(1, 2, 2);
plot(index_union, [max_x, min_x]);
grid on;
legend('max', 'min');
xlabel('index');

end

