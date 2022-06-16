function v = myHermite(x, y, dy, u)
%% Hermite 插值函数，输入任务点坐标参数向量x,y，导数dy和插值点x坐标u，输出插值点y坐标v
if length(x) ~= length(y) || length(x) ~= length(dy)
    error("任务点数据维数不匹配");
end
xy_num = length(x);
uv_num = length(u);
count = 1;
v = zeros(1, uv_num);
for m = 1: xy_num - 1
    while count <= uv_num && u(count) <= x(m + 1)
        v(count) = ( 1+2*(u(count) - x(m))/(x(m+1) - x(m)) )*( (u(count) - x(m+1))/(x(m) - x(m+1)) )^2*y(m)...
            + ( 1+2*(u(count) - x(m+1))/(x(m) - x(m+1)) )*( (u(count) - x(m))/(x(m+1) - x(m)) )^2*y(m+1)...
            + ( u(count) - x(m) )*( (u(count) - x(m+1))/(x(m) - x(m+1)) )^2*dy(m)...
            + ( u(count) - x(m+1) )*( (u(count) - x(m))/(x(m+1) - x(m)) )^2*dy(m+1);
        count = count + 1;
    end
end
end