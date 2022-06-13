function [c, ceq] = myCon(var)
n = (length(var) + 1)/7;
d_tau = var(1:n - 1);
d_theta = reshape(var(n:end), [n, 6]);
global a_max j_max theta
%% 不等号约束条件
c_a = zeros(2*(n - 1), 6);
for num = 1:6
    c_a(:, num) = [-6./d_tau.^2.*theta(1:end - 1, num) + 6./d_tau.^2.*theta(2:end, num) - 4./d_tau.*d_theta(1:end - 1, num) - 2./d_tau.*d_theta(2:end, num);
        6./d_tau.^2.*theta(1:end - 1, num) - 6./d_tau.^2.*theta(2:end, num) + 2./d_tau.*d_theta(1:end - 1, num) + 4./d_tau.*d_theta(2:end, num)];
end
c_j = zeros(n - 2, 6);
for num = 1:6
    c_j(:, num) = c_a(n:2*n - 3, num) - c_a(2:n - 1, num);
end
c = [abs( reshape(c_a, [], 1) ) - a_max; abs( reshape(c_j, [], 1) ) - j_max];
    %% 等号约束条件
ceq = [];
end