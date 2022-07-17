function [cin, ceq] = myCon(var)
n = (length(var) + 13)/7;
d_tau = var(1:n - 1);
d_theta = [zeros(1, 6); reshape(var(n:end), [n - 2, 6]); zeros(1, 6)];
global v_max a_max j_max theta
%% 不等号约束条件
% 时间约束
c_t = -d_tau;
% 速度约束
c_v = reshape( abs(d_theta), [], 1) - v_max;
% 加速度约束
c_a = zeros(2*(n - 1), 6);
for num = 1:6
    c_a(:, num) = [-6./d_tau.^2.*theta(1:end - 1, num) + 6./d_tau.^2.*theta(2:end, num) - 4./d_tau.*d_theta(1:end - 1, num) - 2./d_tau.*d_theta(2:end, num);
        6./d_tau.^2.*theta(1:end - 1, num) - 6./d_tau.^2.*theta(2:end, num) + 2./d_tau.*d_theta(1:end - 1, num) + 4./d_tau.*d_theta(2:end, num)];
end
% 跃度约束
c_j = zeros(n - 2, 6);
for num = 1:6
    c_j(:, num) = c_a(n:2*n - 3, num) - c_a(2:n - 1, num);
end
cin = [c_t; c_v; abs( reshape(c_a, [], 1) ) - a_max; abs( reshape(c_j, [], 1) ) - j_max];
    %% 等号约束条件
ceq = [];
end