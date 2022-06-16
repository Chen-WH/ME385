clc;clear;close all;
%% 运动学测试程序
ur10 = importrobot('ur10.urdf');
% 正向运动学测试
randConfig = ur10.randomConfiguration;
fkine_poe = myfkine_poe([randConfig.JointPosition].')              % poe正向运动学
fkine_rst = getTransform(ur10, randConfig, 'ee_link', 'base_link') % Robotics System Toolbox正向运动学
error = norm(tfm2twist(fkine_poe/fkine_rst));
fprintf("正向运动学：校验通过，误差为%.6f\n", error);

% 逆向运动学
ikine_poe8 = myikine_poe8(fkine_rst)                 % poe正向运动学
ikine_rst = wrapToPi([randConfig.JointPosition].') % 随机生成的关节角度正确值
[error, index] = min(sum( (ikine_poe8 - repmat(ikine_rst, [1, 8]).^2)));
fprintf("逆向运动学：校验通过，误差为%.6f\n", sqrt(error));

% 逆向运动学生成曲线
load('theta_home.mat');
gst0 = myfkine_poe(theta_home.');
point_num = 100;
gst_set = zeros(4, 4, point_num);
gst_set(:, :, 1) = gst0;
for num = 2:point_num
    gst_set(:, :, num) = gst0;
    gst_set(2, 4, num) = gst_set(2, 4, num) + num*0.3/point_num; % 从初始位置连续向y方向运动
end
ikine_poe = myikine_poe(gst_set, theta_home.');
figure(1);
for num = 1:6
    plot(1:point_num, ikine_poe(num, :)); hold on
end
title("逆运动学关节空间曲线");
legend("\theta 1", "\theta 2", "\theta 3", "\theta 4", "\theta 5", "\theta 6", "Location", "best");

% 速度雅可比矩阵判断奇异位形
Js = myJacob0(ikine_poe);
Js_cond = zeros(point_num, 1);
for num = 1:point_num
    Js_cond(num) = cond(Js(:, :, num));
end
figure(2);
plot(Js_cond);
title("速度雅可比矩阵条件数");