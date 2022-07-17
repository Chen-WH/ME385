function A=myAd(T)
R=T(1:3,1:3);p=T(1:3,4);
A=zeros(6,6);
A(1:3,1:3)=R;A(4:6,4:6)=R;
A(4:6,1:3)=skew(p)*R;