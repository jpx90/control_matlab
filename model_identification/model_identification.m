function m = model_identification(y, u, N, init_index)

n = size(y, 1);

P = eye(N) * 1e2;
lambda = 0.95;

m = zeros(n, N);
mm = IIR_with_init(ones(N, 1), init_index, 0)';
mm = mm(N : -1 : 1);
for i = 1 : n
    m(i, :) = mm;
end;

for i = N + 2 : n
    du = u(i - N : i - 1, :) - u(i - N - 1 : i - 2, :);
    dy = y(i) - y(i - N + 1);
    K = P * du / (du' * P * du + lambda);
    m(i, :) = (m(i - 1, :)' + K * (dy - du' * m(i - 1, :)'))';
    P = P - K * du' * P;
    P = P / lambda;
end

end
