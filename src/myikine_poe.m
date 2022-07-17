function theta_set = myikine_poe(gst_set, theta0)
% 逆向运动学函数，生成从给定位置出发得到的关节空间曲线
% input@gst_set：末端位置齐次变换矩阵形式[4, 4, n]
% input@theta0：初始位置关节空间[6, 1]
% output@theta_set：指数坐标法各旋量关节角度[6, n]为解的个数

% 参数设置
n = size(gst_set, 3);
l_gripper = 0.03;
l1 = 0.1273; l2 = 0.2209; l3 = 0.6120; l4 = 0.1719;
l5 = 0.5723; l6 = 0.1149; l7 = 0.1157; l8 = 0.0922 + l_gripper;
gst0 = [0, 1, 0,  l3 + l5;
        1, 0, 0,  l2 - l4 + l6 + l8;
        0, 0, -1, l1 - l7;
        0, 0, 0,  1];

omega = [0, 0, 0, 0, 0,  0;
         0, 1, 1, 1, 0,  1;
         1, 0, 0, 0, -1, 0];
q = [0,  0,  l3,      l3 + l5, l3 + l5,      l3 + l5;
     0,  l2, l2 - l4, l2 - l4, l2 - l4 + l6, l2 - l4 + l6;
     l1, l1, l1,      l1,      l1,           l1 - l7];
zeta = [cross(q, omega); omega];
theta_set = zeros(6, n);

for point = 1:n
    gst = gst_set(:, :, point);
    theta = zeros(6, 8); % 可能有8组解
    %% 求取theta2
    q7 = [q(:, 6); 1]; % 轴6与轴7交点
    g1 = gst/gst0;
    g1q7 = g1*q7;
    A = g1q7(2); B = -g1q7(1); C = q7(2);
    theta(1, 1:4) = 2*atan((B + sqrt(B^2 + A^2 - C^2))/(A+C)); theta(1, 5:8) = 2*atan((B - sqrt(B^2 + A^2 - C^2))/(A+C));
    %% 求取theta6,theta7
    g2 = twist2tfm(zeta(:, 1), -theta(1, 1))*g1;
    R = g2(1:3, 1:3);
    theta(5, 1:2) = acos(R(2, 2)); theta(5, 3:4) = -acos(R(2, 2));
    theta(6, 1:2) = atan2(-R(2, 3), -R(2, 1)); theta(6, 3:4) = atan2(R(2, 3), R(2, 1));
    theta(4, 1:2) = atan2(-R(3, 2), R(1, 2)); theta(4, 3:4) = atan2(R(3, 2), -R(1, 2));
    g2 = twist2tfm(zeta(:, 1), -theta(1, 5))*g1;
    R = g2(1:3, 1:3);
    theta(5, 5:6) = acos(R(2, 2)); theta(5, 7:8) = -acos(R(2, 2));
    theta(6, 5:6) = atan2(-R(2, 3), -R(2, 1)); theta(6, 7:8) = atan2(R(2, 3), R(2, 1));
    theta(4, 5:6) = atan2(-R(3, 2), R(1, 2)); theta(4, 7:8) = atan2(R(3, 2), -R(1, 2));
    %% 求取theta3,theta4,theta5
    for num = 1:4
        g3 = twist2tfm(zeta(:, 1), -theta(1, 2*num - 1))*g1*twist2tfm(zeta(:, 6), -theta(6, 2*num - 1))*twist2tfm(zeta(:, 5), -theta(5, 2*num - 1));
        p = g3(1:3, 4);
        t1 = (l3 + l5)*cos(theta(4, 2*num - 1)) + l1*sin(theta(4, 2*num - 1)) + p(1);
        t2 = l1 - l1*cos(theta(4, 2*num - 1)) + (l3 + l5)*sin(theta(4, 2*num - 1)) - p(3);
        A = 2*t1*l3; B = 2*t2*l3; C = t1^2 + t2^2 + l3^2 - l5^2;
        theta(2, 2*num - 1) = 2*atan((B + sqrt(B^2 + A^2 - C^2))/(A+C)); theta(2, 2*num) = 2*atan((B - sqrt(B^2 + A^2 - C^2))/(A+C));
        theta(3, 2*num - 1) = atan2(t2 - l3*sin(theta(2, 2*num - 1)), t1 - l3*cos(theta(2, 2*num - 1))) - theta(2, 2*num - 1);
        theta(3, 2*num) = atan2(t2 - l3*sin(theta(2, 2*num)), t1 - l3*cos(theta(2, 2*num))) - theta(2, 2*num);
        theta(4, 2*num - 1) = theta(4, 2*num - 1) - theta(2, 2*num - 1) - theta(3, 2*num - 1);
        theta(4, 2*num) = theta(4, 2*num) - theta(2, 2*num) - theta(3, 2*num);
    end
    error = zeros(8, 1);
    for num = 1:8
        error(num) = angleDelta(theta(:, num), theta0);
    end
    [~, index] = min(error);
    theta_set(:, point) = theta(:, index);
    for num = 1:6
        if abs(theta_set(num, point) - theta0(num)) > pi
            theta_set(num, point) = theta_set(num, point) + 2*pi*sign(theta0(num) - theta_set(num, point));
        end
    end
    theta0 = theta_set(:, point);
end
end