function inv_T = invT(T)
% Inverse Matrix of Euler Homogeneous Transformations
% 输入为齐次变换矩阵，输出为其逆矩阵
R_AB = T([1:3],[1:3]); % 提取旋转矩阵
o_AB = T([1:3],[4]); % 提取原点坐标
inv_T = [inv(R_AB) -inv(R_AB)*o_AB; 0 0 0 1];
end