clear
clc
%%
%指数坐标法
t=0:0.1:10;
tau=zeros(3,length(t));
%关节角度，角速度，角加速度
q1=1-cos(t);q2=sin(t.^2);q3=sin(t).^2;
dq1=sin(t);dq2=2*t.*cos(t.^2);dq3=sin(2*t);
ddq1=cos(t);ddq2=2*cos(t.^2)-4*t.^2.*sin(t.^2);ddq3=2*cos(2*t);
q=[q1;q2;q3];
dq=[dq1;dq2;dq3];
ddq=[ddq1;ddq2;ddq3];

omega=[0 0 0;
       0 1 1;
       1 0 0];
p=[0   -0.5 -0.5;
   0   0    0;
   0.5 0    0];
%旋量轴
S=zeros(4,4,3);
s=zeros(6,1,3);
for i=1:3
    vi=cross(p(:,i),omega(:,i));
    S(:,:,i)=[skew(omega(:,i)) ,vi ;0 0 0 0];
    s(:,:,i)=[omega(:,i);vi];
end
%初始位姿
T=zeros(4,4,3);
T(:,:,1)=eye(4);T(1:3,4,1)=[0 0 0.5]'; %T_{1}^{0}
T(:,:,2)=eye(4);T(1:3,4,2)=[0.5 0 0.5]';%T_{2}^{1}
T(:,:,3)=eye(4);T(1:3,4,3)=[1 0 0]';%T_{3}^{2}

g=9.81;
for i=1:length(t)
    V=zeros(6,3);%速度
    dV=zeros(6,3);%加速度
    F=zeros(6,3);%力
    
    for k=1:3
        Sk=S(:,:,k);
        sk=s(:,:,k);
        qk=q(k,i);
        dqk=dq(k,i);
        ddqk=ddq(k,i);
        Tk=T(:,:,k)*expm(Sk*qk);
        invTk=invT(Tk);
        if k==1
            V(:,k)=sk*dqk;
            dV(:,k)=sk*ddqk-my_ad(sk)*V(:,k)*dqk;
        else
            V(:,k)=myAd(invTk)*V(:,k-1)+sk*dqk;
            dV(:,k)=myAd(invTk)*dV(:,k-1)+sk*ddqk-my_ad(sk)*V(:,k)*dqk;
        end
    end
    for k=3:-1:1
        if k==3
            F(:,k)=dV(:,k)-my_ad(V(:,k))'*V(:,k);
        else            
            Sk=S(:,:,k+1);
            sk=s(:,:,k+1);
            qk=q(k+1,i);
            Tk=T(:,:,k+1)*expm(Sk*qk);
            invTk=invT(Tk);
            F(:,k)=myAd(invTk)'*F(:,k+1)+dV(:,k)-my_ad(V(:,k))'*V(:,k);
        end
        RG=eye(3);
        for m=1:k
            Sk=S(:,:,m);
            sk=s(:,:,m);
            qk=q(m,i);
            Tk=T(:,:,m)*expm(Sk*qk);
            Rk=Tk(1:3,1:3);
            RG=RG*Rk;
            Gk=RG'*[0 0 -g]';
        end
            F(:,k)=F(:,k)-[0;0;0;Gk];
            tau(k,i)=dot(s(:,:,k),F(:,k));
    end
end


%%
%工具箱建模
dh(1,:)=[0,       1,   0,  -pi/2, 0];
dh(2,:)=[0,       0,   1,      0, 0];
dh(3,:)=[0,       0,   1,      0, 0];

for i=1:3
    L(i)=Link(dh(i,:));
end
robot = SerialLink(L, 'name', 'robot');

%动力学参数
L(1).m=1;L(1).I=eye(3);L(1).r=[0  0.5  0];L(1).Jm=0;
L(2).m=1;L(2).I=eye(3);L(2).r=[-0.5 0  0];L(2).Jm=0;
L(3).m=1;L(3).I=eye(3);L(3).r=[-0.5 0  0];L(3).Jm=0;

tauDH=robot.rne(q',dq',ddq','gravity',[0 0 9.81]);
tauDH=tauDH';
for i=1:3
    figure(i)
    plot(t,tau(i,:))
    xlabel('t/s')
    ylabel('\tau / Nm')
    title(['第',num2str(i),'号关节的输入力矩(指数坐标递推)'])
end
for i=1:3
    figure(i+3)
    plot(t,tauDH(i,:))
    xlabel('t/s')
    ylabel('\tau / Nm')
    title(['第',num2str(i),'号关节的输入力矩(机器人工具箱)'])
end

