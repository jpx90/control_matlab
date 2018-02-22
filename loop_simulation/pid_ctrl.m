function [y, param] = pid_ctrl(x, param, dt)

param.i = abs_constrain(x * dt, param.i_max);
param.d = param.d + ((x - param.last_x) / dt - param.d) * param.d_iir;
param.last_x = x;

y = + param.kp * x ...
	+ param.ki * param.i ...
	+ param.kd * param.d;

end
