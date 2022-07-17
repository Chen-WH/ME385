load ur10.mat ur10
q = zeros(6,6);
dq = zeros(6,6);
ddq = zeros(6,6);
ur10.DataFormat = 'row';
ur10.Gravity = [0 0 -9.81];
for i = 1:6
    q(i,:) =randomConfiguration(ur10);
    dq(i,:) = q(i,:)*0.5;
    ddq(i,:) = q(i,:)*0.25;
end
tau = ikine2(q,dq,ddq)