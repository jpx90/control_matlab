function model_test

addpath('../lib');

dt = 0.001;
iir_index = 0.001;

t = (0 : dt : 10)';
n = size(t, 1);

input_seq = (t >= 1 & t <= 3) * 1;
output_seq = IIR_with_init(input_seq, iir_index, 0);

m = step_response(iir_index, n);
simu = zeros(n, 1);
x = [0; input_seq];
for i = 1 : n
    d = x(i + 1) - x(i);
    if d ~= 0
        simu(i : n, :) = simu(i : n, :) + m(1 : n - i + 1) * d;
    end;
end;

plot(t, [input_seq, output_seq, simu]);
grid on;

end
