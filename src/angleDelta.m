function del = angleDelta(theta1, theta2)
% This source code is written to calculate the angle between two vectors
del = 0;
for num = 1:length(theta1)
    del = del + abs(acos(dot([cos(theta1(num));sin(theta1(num))],[cos(theta2(num));sin(theta2(num))])));
end
end