function  J = myJacobe(thetaN)
% input@theta：指数坐标法各旋量关节角度[6, N]
% output@J：速度雅可比矩阵[6, 6, N]

% 参数设置
l_gripper = 0.03;
l1 = 0.1273; l2 = 0.2209; l3 = 0.6120; l4 = 0.1719;
l5 = 0.5723; l6 = 0.1149; l7 = 0.1157; l8 = 0.0922 + l_gripper;
N = size(thetaN, 2);
J = zeros(6, 6, N);

omega = [0, 0, 0, 0, 0,  0;
         0, 1, 1, 1, 0,  1;
         1, 0, 0, 0, -1, 0];
q = [0,  0,  l3,      l3 + l5, l3 + l5,      l3 + l5;
     0,  l2, l2 - l4, l2 - l4, l2 - l4 + l6, l2 - l4 + l6;
     l1, l1, l1,      l1,      l1,           l1 - l7];
gst0 = [0, 1, 0,  l3 + l5;
        1, 0, 0,  l2 - l4 + l6 + l8;
        0, 0, -1, l1 - l7;
        0, 0, 0,  1];

xi = [cross(q, omega); omega];
xi_hat=zeros(4,4,6);
for i=1:6
    xi_hat(:,:,i)=[skew(omega(:,i)),cross(q(:,i),omega(:,i));0 0 0 0];
end

for num = 1:N
    theta = thetaN(:, num);
    for i=1:6
        A=gst0;
        for j=6:-1:i
            A=expm(xi_hat(:,:,j)*theta(j))*A;
        end
        A=Ad(invT(A));
        J(:,i,num)=A*xi(:,i);
    end
end