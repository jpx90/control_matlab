function model_test

addpath('../lib');

dt = 0.01;
iir_index1 = 0.10;
iir_index2 = 0.15;
delay_n = 5;
change_n = 400;
N = 50;

t = (0 : dt : 20)';
n = size(t, 1);

% input_seq = (t < 0) .* ((sin(t * 10) > 0.0) * 2 - 1) + (t >= 1) .* sin(t);
input_seq = (sin(t * 5) > 0.5) * 2 - 1;
% input_seq = (t >= 1 & t <= 3) * 1;
% input_seq = sin(t * 2);

output_seq = IIR(input_seq + 0.5, iir_index1);
% output_seq = IIR_with_init(input_seq, iir_index1, 0);
output_seq(change_n : end, :) = IIR_with_init(input_seq(change_n : end, :) + 1, iir_index2, output_seq(change_n - 1));
output_seq = [zeros(delay_n + 1, 1) + output_seq(1); output_seq(1 : end - delay_n - 1, :)];

m = step_response(iir_index2, n);
simu = zeros(n, 1);
x = [0; input_seq];
for i = 1 : n
    d = x(i + 1) - x(i);
    if d ~= 0
        simu(i : n, :) = simu(i : n, :) + m(1 : n - i + 1) * d;
    end;
end;

figure(1);
ax(1) = subplot(1, 2, 1);
[sm, sb] = model_identification(output_seq, input_seq, N, 1);
mesh(1 : N, t, sm);

ax(2) = subplot(1, 2, 2);
plot(1 : N, sm(end, :));
hold on;
plot(delay_n + 1 : N + delay_n, m(1 : N));
hold off;
grid on;
xlim([1, N]);

figure(2);
plot(t, [input_seq, output_seq, sb]);
grid on;

end
