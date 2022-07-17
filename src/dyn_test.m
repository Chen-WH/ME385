clear;clc;
%%
%指数坐标法
t=0:0.1:3;
tau=zeros(6,length(t));
%关节角度，角速度，角加速度
q1=1-cos(t);q2=sin(t);q3=10*cos(t);q4=cos(t);q5=t.^3;q6=t.^3;
dq1=sin(t);dq2=cos(t);dq3=-10*sin(t);dq4=-sin(t);dq5=3*t.^2;dq6=3*t.^2;
ddq1=cos(t);ddq2=-sin(t);ddq3=-10*cos(t);ddq4=-cos(t);ddq5=6*t;ddq6=6*t;
q=[q1;q2;q3;q4;q5;q6];
dq=[dq1;dq2;dq3;dq4;dq5;dq6];
ddq=[ddq1;ddq2;ddq3;ddq4;ddq5;ddq6];

%运动学参数
l1 = 0.1273; l2 = 0.220941; l3 = 0.6120; l4 = 0.1719;
l5 = 0.5723; l6 = 0.1149; l7 = 0.1157; l8 = 0.0922;
xc1 = 0.306; xc2 = 0.28615;

omega=[0 0 0 0 0 0;
       0 1 1 1 0 1;
       1 0 0 0 1 0];

p=[0   0     0      0     0   0     ;
   0   0     0      0     0   0     ;
   0   -xc1  -xc2   0     0   0     ];

%自由度
n=6;
%质量信息: 关节i  质量  质心位置  惯量矩阵[Ixx Iyy Izz Iyz Ixz Ixy]  
%2  7.7780  [0 0 0]  [0.0315 0.0315 0.0219 0 0 0]
%3  12.9300  [0 0 0.3060] [1.6325 1.6325 0.0364 0 0 0]
%4  3.8700  [0 0 0.2862]  [0.4280 0.4280 0.0109 0 0 0]
%5  1.9600 [0 0 0]  [0.0051 0.0051 0.0055 0 0 0]
%6 1.9600  [0 0 0]  [0.0051 0.0051 0.0055 0 0 0]
%7 0.2020  [0 0 0]  [5.2646e-04 5.2646e-04 5.6813e-04 0 0 0]
%质量
m=[7.7780, 12.9300, 3.8700, 1.9600, 1.9600, 0.2020];

%惯量矩阵
I=zeros(3,3,n);
I(:,:,1)=diag([0.0315 0.0315 0.0219]);
I(:,:,2)=diag([1.6325 1.6325 0.0364]);
I(:,:,3)=diag([0.4280 0.4280 0.0109]);
I(:,:,4)=diag([0.0051 0.0051 0.0055]);
I(:,:,5)=diag([0.0051 0.0051 0.0055]);
I(:,:,6)=diag([5.2646e-04 5.2646e-04 5.6813e-04]);

%惯量张量
M=zeros(6,6,n);
for i=1:n
    M(1:3,1:3,i)=I(:,:,i);
    M(4:6,4:6,i)=m(i)*eye(3);
end

%旋量轴
S=zeros(4,4,n);
s=zeros(6,1,n);
for i=1:n
    vi=cross(p(:,i),omega(:,i));
    S(:,:,i)=[skew(omega(:,i)) ,vi ;0 0 0 0];
    s(:,:,i)=[omega(:,i);vi];
end

%初始位姿
T=zeros(4,4,n);
T(:,:,1)=eye(4);T(1:3,4,1)=[0 0 l1]'; 
T(:,:,2)=[0 0 1  xc1;
          0 1 0  l2;
         -1 0 0  0;
          0 0 0  1];
T(:,:,3)=eye(4);T(1:3,4,3)=[0 -l4 xc2+l3-xc1]';
T(:,:,4)=[0 0 1  0;
          0 1 0  0;
         -1 0 0  l5-xc2;
          0 0 0  1];
T(:,:,5)=eye(4);T(1:3,4,5)=[0 l6 0]';
T(:,:,6)=eye(4);T(1:3,4,6)=[0 0  l7]';     
      
g=9.81;
for i=1:length(t)
    V=zeros(6,n);%速度
    dV=zeros(6,n);%加速度
    F=zeros(6,n);%力
    
    for k=1:n
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
    for k=n:-1:1
        if k==n
            F(:,k)=M(:,:,k)*dV(:,k) - my_ad(V(:,k))'*M(:,:,k)*V(:,k);
        else            
            Sk=S(:,:,k+1);
            sk=s(:,:,k+1);
            qk=q(k+1,i);
            Tk=T(:,:,k+1)*expm(Sk*qk);
            invTk=invT(Tk);
            F(:,k)=myAd(invTk)'*F(:,k+1) + M(:,:,k)*dV(:,k) - my_ad(V(:,k))'*M(:,:,k)*V(:,k);
        end
        RG=eye(3);
        for j=1:k
            Sk=S(:,:,j);
            sk=s(:,:,j);
            qk=q(j,i);
            Tk=T(:,:,j)*expm(Sk*qk);
            Rk=Tk(1:3,1:3);
            RG=RG*Rk;
            Gk=RG'*[0 0 -m(k)*g]';
        end
            F(:,k)=F(:,k)-[0;0;0;Gk];
            tau(k,i)=dot(s(:,:,k),F(:,k));
    end
end

tau_box = ikine2(q',dq',ddq');
tau_box = tau_box';
norm(tau(3,:)- tau_box(3,:))/length(t)

for i=1:n
    figure(i)
    plot(t,tau(i,:),'r-',t,tau_box(i,:),'b-')
    xlabel('t/s')
    ylabel('\tau / Nm')
    title(['第',num2str(i+1),'号关节的输入力矩'])
    legend('指数坐标递推','inverseDynamics')
 end
