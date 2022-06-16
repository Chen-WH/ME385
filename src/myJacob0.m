function  J=myJacob0(thetaN)
% input@theta：指数坐标法各旋量关节角度[6, N]
% output@J：速度雅可比矩阵[6, 6, N]
N = size(thetaN, 2);
J = zeros(6,6,N);
for num = 1:N
    theta = thetaN(:, num);
    T=myfkine_poe(theta);
    R=T(1:3,1:3);
    rot=[R,zeros(3);zeros(3),R];
    J(:, :,num)=rot*myJacobe(theta);
end
end