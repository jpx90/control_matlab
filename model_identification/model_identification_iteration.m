function [m, b, P] = model_identification_iteration(y, u, du, m, b, P, N, lambda)

dy = y - u;
H = [du; 1];
K = P * H / (H' * P * H + lambda);
dx = K * (dy - (du' * m + b));
m = m + dx(1 : N);
b = b + dx(N + 1);
P = P - K * H' * P;
P = P / lambda;

end
