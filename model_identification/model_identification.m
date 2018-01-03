function [m, b] = model_identification(y, u, N, init_index)

n = size(y, 1);

P = diag([ones(N, 1) * 1e2; 1e0]);
lambda = 0.995;

m = zeros(n, N);
b = zeros(n, 1);
mm = IIR_with_init(ones(N, 1), init_index, 0)';
for i = 1 : n
    m(i, :) = mm;
end;

for i = N + 2 : n
    du = u(i - 1 : -1 : i - N, :) - u(i - 2 : -1 : i - N - 1, :);
    dy = y(i) - u(i - N - 1);
	H = [du; 1];
    K = P * H / (H' * P * H + lambda);
	dx = K * (dy - (du' * m(i - 1, :)' + b(i - 1)));
    m(i, :) = m(i - 1, :) + dx(1 : N)';
	b(i) = b(i - 1) + dx(N + 1);
    P = P - K * H' * P;
    P = P / lambda;
end

end
