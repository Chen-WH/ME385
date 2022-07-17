%基于rigidbodytree的逆动力学
function [tau] = ikine2(q,dq,ddq)
    %加载机器人本体
    ur10 = importrobot('ur10.urdf');
    ur10.DataFormat = 'row';
    ur10.Gravity = [0 0 -9.81];
    %设置力矩参数
    tau = zeros(length(q(:,1)),6);
    %返回力矩参数
    for i = 1:length(q(:,2))
        tau(i,:) = inverseDynamics(ur10,q(i,:),dq(i,:),ddq(i,:));
    end
end