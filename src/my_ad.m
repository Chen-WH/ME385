function A=my_ad(s)
w=s(1:3,1);
v=s(4:6,1);
A=zeros(6,6);
A(1:3,1:3)=skew(w);
A(4:6,4:6)=skew(w);
A(4:6,1:3)=skew(v);