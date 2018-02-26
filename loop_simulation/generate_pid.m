function p = generate_pid(kp, ki, kd, i_max, d_iir)

p.kp = kp;
p.ki = ki;
p.kd = kd;
p.i_max = i_max;
p.d_iir = d_iir;
p.i = 0;
p.d = 0;
p.last_x = 0;

end
