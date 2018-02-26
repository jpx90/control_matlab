if 0
	act_to_acc_iir_index = 0.01;
	act_to_acc_delay_n = 50;
	act_to_acc_sim_iir_index = 0.01;
	
	pos_kp = 2;
	vel_kp = 5;
	ang_kp = 2.35;
	
	pid_ang_vel = generate_pid(5.52, 0, 0, 0, 0);
	
	ang_N = 25;
	ang_M = 20;
	ang_Q = diag([ones(10, 1) * 0; ones(15, 1)]);
	ang_R = eye(ang_M) * 1;
end

if 0
	act_to_acc_iir_index = 0.06;
	act_to_acc_delay_n = 30;
	act_to_acc_sim_iir_index = 0.043;
	
	pos_kp = 2;
	vel_kp = 5;
	ang_kp = 5.6;
	
	pid_ang_vel = generate_pid(12.3, 0, 0, 0, 0);
	
	ang_N = 30;
	ang_M = 20;
	ang_Q = diag([ones(10, 1) * 0; ones(20, 1)]);
	ang_R = eye(ang_M) * 1;
	
% 	ang_N2 = 25;
% 	ang_M2 = 20;
% 	ang_Q2 = diag([ones(9, 1) * 0; ones(16, 1)]);
% 	ang_R2 = eye(ang_M2) * 1;
end

if 1
	act_to_acc_iir_index = 0.06;
	act_to_acc_delay_n = 20;
	act_to_acc_sim_iir_index = 0.048;
	
	pos_kp = 2;
	vel_kp = 5;
	ang_kp = 7.2;
	
	pid_ang_vel = generate_pid(15.4, 0, 0, 0, 0);
	
	ang_N = 20;
	ang_M = 15;
	ang_Q = diag([ones(8, 1) * 0; ones(12, 1)]);
	ang_R = eye(ang_M) * 1;
	
% 	ang_N2 = 30;
% 	ang_M2 = 20;
% 	ang_Q2 = diag([ones(8, 1) * 0; ones(22, 1)]);
% 	ang_R2 = eye(ang_M2) * 1;
end