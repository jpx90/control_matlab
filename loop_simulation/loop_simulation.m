% function loop_simulation

clear all

addpath('../lib');
addpath('../model_identification');

max_n = 50000;
dt_s = 0.001;
t = (1 : max_n)' * dt_s;

GRAVITY = 9.81;

configuration;
declare_state;
set_pid;
set_target;

sim_act = zeros(max_n, 1);

%% Model

act_to_acc_iir_index = 0.01;
act_to_acc_delay_n = 50;
act_to_acc_sim_iir_index = 0.009;

noise_freq = 230;
ave_n = 87;
ave_n2 = 43;
noise_ang_vel = sin(t * 2 * pi * noise_freq) * 1;
noise_ang = -cos(t * 2 * pi * noise_freq) * 1 / 2 / pi / noise_freq;
res_ang_vel = 0;

%% Angle Model Identification
ang_step = 100;
ang_MN = 50;
ang_init_index = 1;
ang_P = diag([ones(ang_MN, 1) * 1e2; 1e0]);
ang_lambda = 0.995;
ang_m = IIR_with_init(ones(ang_MN, 1), ang_init_index, 0);
ang_b = 0;
ang_m_hist = zeros(0, ang_MN);

%% Angular Velocity Model Identification
ang_vel_step = 50;
ang_vel_MN = 50;
ang_vel_init_index = 1;
ang_vel_P = diag([ones(ang_vel_MN, 1) * 1e2; 1e0]);
ang_vel_lambda = 0.995;
ang_vel_m = IIR_with_init(ones(ang_vel_MN, 1), ang_vel_init_index, 0);
ang_vel_b = 0;
ang_vel_m_hist = zeros(0, ang_vel_MN);

%% MPC
tick_mpc1 = 28000;
tick_mpc2 = 38000;

ang_N1 = 40;
ang_M1 = 30;
ang_Q1 = diag([ones(20, 1) * 0; ones(20, 1)]);
ang_R1 = eye(ang_M1) * 0.1;

% ang_N2 = 30;
% ang_M2 = 20;
% ang_Q2 = diag([ones(8, 1) * 0; ones(22, 1)]);
% ang_R2 = eye(ang_M2) * 0.1;

ang_N2 = 20;
ang_M2 = 15;
ang_Q2 = diag([ones(8, 1) * 0; ones(12, 1)]);
ang_R2 = eye(ang_M2) * 0.1;

ang_predict = zeros(max_n, 1);
ang_cmd = zeros(max_n, 1);

%% Loop

for i = ang_step * (ang_MN + 2) + 1 : max_n - ang_MN * ang_step
	ang_acc(i) = act(i - act_to_acc_delay_n) * act_to_acc_iir_index + ang_acc(i - 1) * (1 - act_to_acc_iir_index);
	ang_vel(i) = ang_vel(i - 1) + ang_acc(i) * dt_s;
	ang(i) = ang(i - 1) + ang_vel(i) * dt_s;
	acc(i) = tan(ang(i)) * GRAVITY;
	vel(i) = vel(i - 1) + acc(i) * dt_s;
	pos(i) = pos(i - 1) + vel(i) * dt_s;
	
	res_ang_vel = res_ang_vel * 0.9 + (ang_vel(i) + noise_ang_vel(i)) * 0.1;
	
	% 	tar_vel(i) = abs_constrain((tar_pos(i) - pos(i)) * pos_kp, 5);
	% 	tar_acc(i) = abs_constrain((tar_vel(i) - vel(i)) * vel_kp, 5);
	% 	tar_ang(i) = atan(tar_acc(i) / GRAVITY);
	% 	tar_ang_vel(i) = abs_constrain((tar_ang(i) - ang(i) - noise_ang(i)) * ang_kp, 5);
	
% 	if i == 11000
% 		tar_ang(30000 : 32000) = -ang(9000 : 11000);
% 	end
	
	if mod(i, ang_step) == 1
		pas_index = i - (1 : ang_MN) * ang_step - ang_step;
		du = ang_cmd(pas_index) - ang_cmd(pas_index - ang_step);
		if i < tick_mpc1
			temp = i - ang_step + (-ave_n2 : ave_n2);
			[ang_m, ang_b, ang_P] = model_identification_iteration( ...
				sum(ang(temp) + noise_ang(temp)) / ave_n, ...
				ang_cmd(i - ang_step * (ang_MN + 2)), ...
				du, ang_m, ang_b, ang_P, ang_MN, ang_lambda);
			ang_m_hist = [ang_m_hist; ang_m'];
		end
		
		du = ang_cmd(i - ang_step) - ang_cmd(i - ang_step * 2);
		pre_index = (0 : ang_MN - 1) * ang_step + i;
		ang_predict(pre_index(end)) = ang_predict(pre_index(end - 1));
		ang_predict(pre_index) = ang_predict(pre_index) + du * ang_m;
		ang_predict(pre_index) = ang_predict(pre_index) + ang(i) - ang_predict(i);
		
		if i < tick_mpc2
			OA = zeros(ang_N1);
			for j = 1 : ang_N1
				OA(j, 1 : j) = ang_m(j : -1 : 1);
			end;
			A = OA(1 : ang_N1, 1 : ang_M1);
			xxx = A' * ang_Q1;
			xxx = (xxx * A + ang_R1) \ xxx;
			ang_N = ang_N1;
		else
			OA = zeros(ang_N2);
			for j = 1 : ang_N2
				OA(j, 1 : j) = ang_m(j : -1 : 1);
			end;
			A = OA(1 : ang_N2, 1 : ang_M2);
			xxx = A' * ang_Q2;
			xxx = (xxx * A + ang_R2) \ xxx;
			ang_N = ang_N2;
		end
		
		pre_index = (0 : ang_N - 1) * ang_step + i;
		
		if i > 280000
			du_p = xxx * (tar_ang(pre_index) - ang_predict(pre_index));
		else
			du_p = xxx * (tar_ang(i) * ones(ang_N, 1) - ang_predict(pre_index));
		end
		
		if i > tick_mpc1
			if set_ang_ctrl_smooth
				ang_cmd(i) = ang_cmd(i - 1) + du_p(1) / ang_step;
			else
				ang_cmd(i) = ang_cmd(i - ang_step) + du_p(1);
			end
		else
			if set_ang_ctrl_smooth
				du_p = tar_ang(i) - ang_cmd(i - 1);
				ang_cmd(i) = ang_cmd(i - 1) + du_p(1) / ang_step;
			else
				ang_cmd(i) = tar_ang(i);
			end
		end
		
		tar_ang_vel(i) = abs_constrain((ang_cmd(i) - ang(i) - noise_ang(i)) * ang_kp, 5);
	else
		if set_ang_ctrl_smooth
			temp = mod(i, ang_step);
			if temp == 0
				temp = ang_step;
			end
			ang_cmd(i) = ang_cmd(i - temp) + du_p(1) * temp / ang_step;
			
			tar_ang_vel(i) = abs_constrain((ang_cmd(i) - ang(i) - noise_ang(i)) * ang_kp, 5);
		else
			ang_cmd(i) = ang_cmd(i - 1);
			tar_ang_vel(i) = tar_ang_vel(i - 1);
		end
		pre_index = (0 : ang_N - 1) * ang_step + i;
		ang_predict(pre_index) = ang_predict(pre_index - 1);
	end;
	
	if mod(i, ang_vel_step) == 1
		temp = ceil(ave_n2 / ang_vel_step);
		pas_index = i - ((1 : ang_vel_MN) + temp) * ang_vel_step;
		du = tar_ang_vel(pas_index) - tar_ang_vel(pas_index - ang_vel_step);
		if i < tick_mpc2
			temp2 = i - ang_vel_step * temp + (-ave_n2 : ave_n2);
			[ang_vel_m, ang_vel_b, ang_vel_P] = model_identification_iteration( ...
				sum(ang_vel(temp2) + noise_ang_vel(temp2)) / ave_n, ...
				tar_ang_vel(i - ang_vel_step * (ang_vel_MN + 1 + temp)), ...
				du, ang_vel_m, ang_vel_b, ang_vel_P, ang_vel_MN, ang_vel_lambda);
			ang_vel_m_hist = [ang_vel_m_hist; ang_vel_m'];
		end
	end
	
	[temp, pid_ang_vel] = pid_ctrl(tar_ang_vel(i) - res_ang_vel, pid_ang_vel, dt_s);
	tar_ang_acc(i) = abs_constrain(temp, 100);
	act(i) = tar_ang_acc(i);
% 	act(i) = (1.5 * tar_ang_acc(i) - sim_act(i - 1)) * 2;
	sim_act(i) = act(i) * act_to_acc_sim_iir_index + sim_act(i - 1) * (1 - act_to_acc_sim_iir_index);
end;

max_n = max_n - ang_N * ang_step;
t = t(1 : max_n);
pos = pos(1 : max_n);
vel = vel(1 : max_n);
acc = acc(1 : max_n);
ang = ang(1 : max_n);
ang_vel = ang_vel(1 : max_n);
ang_acc = ang_acc(1 : max_n);
act = act(1 : max_n);
noise_ang = noise_ang(1 : max_n);
noise_ang_vel = noise_ang_vel(1 : max_n);

tar_pos = tar_pos(1 : max_n);
tar_vel = tar_vel(1 : max_n);
tar_acc = tar_acc(1 : max_n);
tar_ang = tar_ang(1 : max_n);
tar_ang_vel = tar_ang_vel(1 : max_n);
tar_ang_acc = tar_ang_acc(1 : max_n);

ang_cmd = ang_cmd(1 : max_n);

ang_predict = ang_predict(1 : max_n);

figure(1);
ax(1) = subplot(3, 2, 1); plot(t, [tar_pos, pos]); grid on; legend('tar pos', 'pos');
ax(2) = subplot(3, 2, 3); plot(t, [tar_vel, vel]); grid on; legend('tar vel', 'vel');
ax(3) = subplot(3, 2, 5); plot(t, [tar_acc, acc]); grid on; legend('tar acc', 'acc');
ax(4) = subplot(3, 2, 2); plot(t, [tar_ang, ang, ang_cmd]); grid on; legend('tar ang', 'ang', 'ang cmd');
ax(5) = subplot(3, 2, 4); plot(t, [tar_ang_vel, ang_vel]); grid on; legend('tar ang vel', 'ang vel');
ax(6) = subplot(3, 2, 6); plot(t, [tar_ang_acc, ang_acc]); grid on; legend('tar ang acc', 'ang acc');

linkaxes(ax, 'x');
xlim([29, 37]);

figure(2);
% step = ang_step / 2;
% [m, ~] = model_identification(ang_vel(1 : step : tick_mpc2), tar_ang_vel(1 : step : tick_mpc2), 100, 1);
% subplot(1, 2, 1); mesh(ang_m_hist); grid on;
subplot(1, 2, 1); plot(ang_vel_m); grid on; title('ang vel');
subplot(1, 2, 2); plot(ang_m); grid on; title('ang');

% figure(3);
% clear ax;
% subplot(1, 2, 1); mesh(m); grid on;

% end