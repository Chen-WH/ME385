%% Hermite 插值轨迹规划
%input@theta：关节空间任务点[n, 6]
%input@N：得到的总插值点个数
%input@v_max：最大角速度约束
%input@a_max：最大角加速度约束
%input@j_max：最大角跃度约束
%output@t：插值的时间[N, 1]
%output@x：插值点的角度[N, 6]
%output@v：插值点的角速度[N, 6]
%output@a：插值点的角加速度[N, 6]
function [t, x, tau] = trajectory_Her(theta, N, v_max, a_max, j_max)
% 规划量为任务点对应 d_tau 和任务点对应 d_theta
% var = [d_tau, d_theta]，其中 d_tau[1, n - 1] d_theta[1, 6n]
n = size(theta, 1);
%% 规划
f = @(x) sum(x(1:n - 1)); % 代价函数，这里定义为最后一个任务点对应时间
x0 = rand(7*n - 1, 1);    % 初始值 d_tau 和 d_theta 均随机数生成
options = optimoptions('fmincon','Algorithm','sqp');
var = fmincon(f, x0, [], [], [], [], [], [], 'myCon', options); % fmincon求解
% var = mySQP(f, x0, @myCon); % SQP求解

%% 插值
d_tau = var(1:n - 1);
tau = [0; d_tau];
for num = 2:n
    tau(num) = tau(num - 1) + tau(num);
end
d_theta = reshape(var(n:end), [n, 6]);
t = linspace(0, sum(d_tau), N);
x = zeros(N, 6);
for num = 1:6
    x(:, num) = myHermite(tau, theta(:, num), d_theta(:, num), t);
end
end