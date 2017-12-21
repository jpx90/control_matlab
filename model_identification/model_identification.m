function m = model_identification(y, u, N, init_index)

n = size(y, 1);

P = eye(N) * 1e2;
lambda = 0.995;

m = zeros(n, N);
mm = IIR_with_init(ones(N, 1), init_index, 0)';
for i = 1 : n
    m(i, :) = mm;
end;

for i = N + 2 : n
    du = u(i - 1 : -1 : i - N, :) - u(i - 2 : -1 : i - N - 1, :);
    dy = y(i) - u(i - N - 1);
    K = P * du / (du' * P * du + lambda);
    m(i, :) = (m(i - 1, :)' + K * (dy - du' * m(i - 1, :)'))';
    P = P - K * du' * P;
    P = P / lambda;
end

end
