function gst = myfkine_poe(theta)
% 正向运动学函数
% input@theta：指数坐标法各旋量关节角度[6, N]
% output@gst：末端位置齐次变换矩阵形式[4, 4, N]

% 参数设置
l_gripper = 0.03;
l1 = 0.1273; l2 = 0.2209; l3 = 0.6120; l4 = 0.1719;
l5 = 0.5723; l6 = 0.1149; l7 = 0.1157; l8 = 0.0922 + l_gripper;
gst0 = [0, 1, 0,  l3 + l5;
        1, 0, 0,  l2 - l4 + l6 + l8;
        0, 0, -1, l1 - l7;
        0, 0, 0,  1];

N = size(theta, 2); % 获取位姿点个数
gst = zeros(4, 4, N); % 初始化

omega = [0, 0, 0, 0, 0,  0;
         0, 1, 1, 1, 0,  1;
         1, 0, 0, 0, -1, 0];
q = [0,  0,  l3,      l3 + l5, l3 + l5,      l3 + l5;
     0,  l2, l2 - l4, l2 - l4, l2 - l4 + l6, l2 - l4 + l6;
     l1, l1, l1,      l1,      l1,           l1 - l7];
zeta = [cross(q, omega); omega];

for count = 1:N
    gst(:, :, count) = eye(4);
    for joint = 1:6
        gst(:, :, count) = gst(:, :, count)*twist2tfm(zeta(:, joint), theta(joint, count));
    end
    gst(:, :, count) = gst(:, :, count)*gst0;
end
end