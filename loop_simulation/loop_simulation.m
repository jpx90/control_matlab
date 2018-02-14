function loop_simulation

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

act_to_acc_iir_index = 0.06;
act_to_acc_delay_n = 20;
act_to_acc_sim_iir_index = 0.045;

noise_freq = 250;
noise_ang_vel = sin(t * 2 * pi * noise_freq) * 1;
noise_ang = -cos(t * 2 * pi * noise_freq) * 1 / 2 / pi / noise_freq;
res_ang_vel = 0;

%% Angle Model Identification
ang_step = 20;
ang_MN = 50;
ang_init_index = 1;
ang_P = diag([ones(ang_MN, 1) * 1e2; 1e0]);
ang_lambda = 0.995;
ang_m = IIR_with_init(ones(ang_MN, 1), ang_init_index, 0);
ang_b = 0;
ang_m_hist = zeros(0, ang_MN);

%% MPC
tick_mpc1 = 10000;
tick_mpc2 = 30000;

ang_N = 20;
ang_M = 15;
ang_Q = diag([ones(15, 1) * 0; ones(5, 1)]);
ang_M2 = 15;
ang_Q2 = diag([ones(10, 1) * 0; ones(10, 1)]);
% ang_Q2 = diag([ones(5, 1) * 0; (1 : 5)' / 5; ones(40, 1)]);
% ang_R = diag([0.01, ones(1, ang_M - 1)]) * 1;
ang_R = eye(ang_M) * 0.1;
ang_R2 = eye(ang_M2) * 0.1;
ang_predict = zeros(max_n, 1);
ang_cmd = zeros(max_n, 1);

%% Loop

for i = ang_step * (ang_MN + 1) + 1 : max_n - ang_MN * ang_step
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

    if mod(i, ang_step) == 1
        pas_index = i - (1 : ang_MN) * ang_step;
        du = ang_cmd(pas_index) - ang_cmd(pas_index - ang_step);
        if i < 30000
            [ang_m, ang_b, ang_P] = model_identification_iteration( ...
                ang(i), ...
                ang_cmd(i - ang_step * (ang_MN + 1)), ...
                du, ...
                ang_m, ...
                ang_b, ...
                ang_P, ...
                ang_MN, ...
                ang_lambda);
            ang_m_hist = [ang_m_hist; ang_m'];
        end
        
        OA = zeros(ang_N);
        for j = 1 : ang_N
            OA(j, 1 : j) = ang_m(j : -1 : 1);
        end;
		if i < tick_mpc2
			A = OA(1 : ang_N, 1 : ang_M);
			xxx = A' * ang_Q;
			xxx = (xxx * A + ang_R) \ xxx;
		else
			A = OA(1 : ang_N, 1 : ang_M2);
			xxx = A' * ang_Q2;
			xxx = (xxx * A + ang_R2) \ xxx;
		end
        
        pre_index = (0 : ang_N - 1) * ang_step + i;
        ang_predict(pre_index(end)) = ang_predict(pre_index(end - 1));
        ang_predict(pre_index) = ang_predict(pre_index) + du(1) * ang_m(1 : ang_N);
        ang_predict(pre_index) = ang_predict(pre_index) + ang(i) - ang_predict(i);
        
        if i > 27000
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
            ang_cmd(i) = tar_ang(i);
        end
        
        tar_ang_vel(i) = abs_constrain((ang_cmd(i) - ang(i) - noise_ang(i)) * ang_kp, 5);
    else
        if i > tick_mpc1 && set_ang_ctrl_smooth
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
    
	tar_ang_acc(i) = abs_constrain((tar_ang_vel(i) - res_ang_vel) * ang_vel_kp, 100);
%     act(i) = tar_ang_acc(i);
    act(i) = (1.5 * tar_ang_acc(i) - sim_act(i - 1)) * 2;
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

figure(2);
% step = ang_step;
% [m, ~] = model_identification(ang(1 : step : end), tar_ang(1 : step : end), 50, 1);
% clear ax;
subplot(1, 2, 1); mesh(ang_m_hist); grid on;
subplot(1, 2, 2); plot(ang_m); grid on;

end