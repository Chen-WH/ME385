function R = myRodrigues(omega, theta)
%% 罗德里格斯公式，输入为旋转向量omega和旋转角度theta
Omega = [0, -omega(3), omega(2);
    omega(3), 0, -omega(1);
    -omega(2), omega(1), 0];
R = eye(3) + sin(theta)*Omega + (1 - cos(theta))*Omega^2;
end