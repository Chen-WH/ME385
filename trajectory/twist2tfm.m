function H = twist2tfm(zeta, theta)
%% 运动旋量转换为齐次变换矩阵函数，输入为运动旋量zeta和关节变量theta
v = zeta(1:3); omega = zeta(4:6);
if norm(omega) ~= 0 % 旋转关节
    H = [myRodrigues(omega, theta), (eye(3) - myRodrigues(omega, theta))*cross(omega, v) + dot(omega, v)*omega*theta;
        0, 0, 0, 1];
else
     H = [eye(3), v*theta;
         0, 0, 0, 1];
end
end