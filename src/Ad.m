function Ad = Ad(T)
% given g, calculate Ad_g
R = T(1:3,1:3);
p_hat = [0 -T(3, 4) T(2, 4);
    T(3, 4) 0 -T(1, 4);
    -T(2, 4) T(1, 4) 0]*R;
Ad = [R p_hat; zeros(3,3) R];
end