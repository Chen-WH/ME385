clear
clc
xend=150+75;
yend=500;
x=600:-1:-xend;
tan_alpha=yend/(600+xend);
y=tan_alpha*(x-600);
delta=50;
% h=( -(150^2-delta^2) + sqrt((150^2-delta^2)^2 + 4*delta^2) )/2;
% h=sqrt(h);
beta=asin(delta/sqrt( (xend+450)^2+860^2 ));
gamma=atan2(860,xend+450);

len=sqrt( (xend+450)^2+860^2 - delta^2 )*cos(beta+gamma);
z=(-860 + tan(beta+gamma)*(x+xend)).*(x+xend<len) +  sqrt( delta^2-(450-x).^2 ).*(x+xend>len & x<450)  + delta.*(x>=450);

x=x/1000;y=y/1000;z=z/1000;

figure(1)
plot3(x,y,z)
xlabel('x')
ylabel('y')
zlabel('z')
title("逆运动学工作空间路径");
grid on
n=length(x);
g=zeros(4,4,n);
for i=1:n
    g(:,:,i)=[0  1  0   x(i);
              0  0 -1   y(i);
             -1  0  0   z(i);
              0  0  0    1  ];
end

theta0=myikine_poe8(g(:,:,1));
theta_home=theta0(:,4);
ur10 = importrobot('ur10.urdf');

theta_traj = myikine_poe(g, theta_home);
config= homeConfiguration(ur10);
for n=1:20:800%length(theta_traj)
    figure(2)
    for i=1:6
        config(i).JointPosition = theta_traj(i,n);
    end
    show(ur10,config)
    drawnow;
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if n == 1
        imwrite(imind,cm, "demo.gif",'gif', 'Loopcount',inf,'DelayTime',0.01);
    else
        imwrite(imind,cm, "demo.gif",'gif','WriteMode','append','DelayTime',0.01);
    end
    clf;

end

Js = myJacob0(theta_traj);
Js_cond = zeros(size(Js, 3), 1);
for num = 1:size(Js, 3)
    Js_cond(num) = cond(Js(:, :, num));
end
singular_index = find(Js_cond > 10);

for n=1:20:800%length(theta_traj)
    figure(3)
    for i=1:6
        config(i).JointPosition = theta_traj(i,n);
    end
    show(ur10,config)
    hold on
    scatter3(g(1, 4, singular_index), g(2, 4, singular_index), g(3, 4, singular_index), 'r');
    drawnow;
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if n == 1
        imwrite(imind,cm, "demo1.gif",'gif', 'Loopcount',inf,'DelayTime',0.01);
    else
        imwrite(imind,cm, "demo1.gif",'gif','WriteMode','append','DelayTime',0.01);
    end
    clf;

end