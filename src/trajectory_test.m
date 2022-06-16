clc;clear;close all;
global v_max a_max j_max theta
v_max = pi;
a_max = 1;
j_max = 1;
ur10 = importrobot('ur10.urdf');

% 逆向运动学生成曲线
load('theta_home.mat');
gst0 = myfkine_poe(theta_home.');
point_num = 20;
gst_set = zeros(4, 4, point_num);
gst_set(:, :, 1) = gst0;
for num = 2:point_num
    gst_set(:, :, num) = gst0;
    gst_set(3, 4, num) = gst_set(2, 4, num) - num*0.5/point_num; % 从初始位置连续向z轴负方向运动
end
theta = myikine_poe(gst_set, theta_home.').';

% 轨迹规划
[t, x, tau] = trajectory_Her(theta, 500, v_max, a_max, j_max);
figure(1);
plot(t, x(:, 1), t, x(:, 2), t, x(:, 3), t, x(:, 4), t, x(:, 5), t, x(:, 6)); hold on
for num = 1:6
    scatter(tau, theta(:, num));
end
title("关节空间轨迹规划图");
xlabel("时间(s)"); ylabel("关节角度(rad)");
legend("插值曲线", "", "" , "" , "", "", "任务点", "Location", "best");