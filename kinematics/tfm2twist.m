function twist = tfm2twist(H)
%% 齐次变换矩阵转换为运动旋量函数
R = H(1:3, 1:3);
if trace(R) == 3
    omega = 0;
elseif trace(R) == -1
    theta = pi;
    omega_multi = sqrt(R(1, 2)*R(1, 3)*R(2, 3)/2);
    omega = [omega_multi/R(2, 3); omega_multi/R(1, 3); omega_multi/R(1, 2)]*theta;
else
    theta = acos( (trace(R) - 1)/2 );
    omega = [R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)]/2/sin(theta)*theta;
end
if norm(omega) < 1e-7
    v = H(1:3, 4);
else
    omega_bar = [0, -omega(3), omega(2);
        omega(3), 0, -omega(1);
        -omega(2), omega(1), 0];
    A = (eye(3) - R)*omega_bar/theta + omega*omega.'/theta^2;
    v = A\H(1:3, 4);
end
twist = [v; omega];
end