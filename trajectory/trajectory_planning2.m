%input@x,y,z：任务点坐标
%input@N：得到的总插值点个数
%input@vmax：最大速度约束
%input@amax：最大加速度约束
%output@t：到达插值后每一个任务点的时间[N, 1]
%output@x：插值后每一个任务点的坐标[N, 3]
%output@v：插值后每一个任务点的速度[N, 3]
%output@a：插值后每一个任务点的加速度[N, 3]
%output@T：任务点的对应时刻
function [T, t, x, y, z, vx, vy, vz, ax, ay, az] = task3(x, y, z, N, vmax, amax)
    C = [x,y,z];
    %反解控制点
    C = [0, 0, 0; C];
    [P, u_C] = bSplineInterpolation(C, 3);
    %生成B样条曲线
    u = linspace(0,0.99,N);
    res = bSpline(P, u);
    
    %更新C到插值后的点
    C = res;
    iter_num = 1e+6;
    dudt = myprogram(C, u, vmax, amax, iter_num);
    
    [t, v, a] = get_tva(dudt, C, u);
    v(end + 1, :) = v(end, :);
    a(end + 1, :) = a(end, :);
    a(end + 1, :) = a(end, :);
    T = zeros([length(u_C), 1]);
    for i = 1:length(u_C)
        index = find(u >= u_C(i));
        index = index(1);
        T(i) = t(index);
    end
    coord = C;
    z = coord(:, 3); y = coord(:, 2); x = coord(:, 1);
    vx = v(:, 1); vy = v(:, 2); vz = v(:, 3);
    ax = a(:, 1); ay = a(:, 2); az = a(:, 3);
end


%% functions
%输入：给定的数据点P(n, 3), B样条次数p
%输出：控制点
function [res, u] = bSplineInterpolation(C, p)
    k = size(C, 1);
    %控制点数目等于给定点的数目
    n = k;
    %选取的节点数目
    m = n + p + 1;
    %在0~1上选取m个节点
    U = getU(m, n, 1);    
    %取u
    u = linspace(0,0.99, k);

    %反解矩阵A
    A = zeros([n,n]);
    for i = 1:n
        for j = 1:n
            A(i, j) = N(u(i), j - 1, U, p);
        end
    end
    res = A\C;
end

%输入：m个节点，n个控制点，
%          type==1，准均匀节点矢量
%输出：U(1, m)
function res = getU(m, n, type)
    p = m - n - 1;
    res = zeros(1, m);
    if type == 1
        tmp = linspace(0, 1, n)';
        res(1: p) = 0;
        res(m - p+1: m) = 1;
        res(p+1:m-p) = linspace(0,1, m - 2 * p);
    end
end

% 三阶b样条
function res = bSpline(P, u)
    %P控制点 u x粒度
    p = 3;
    n = size(P,1);
    m = n + p + 1;
    res = zeros([size(u,2),3]);
    %U = linspace(0, 1, m);
    U(1: p) = 0;
    U(m - p+1: m) = 1;
    U(p+1:m-p) = linspace(0,1, m - 2 * p);
    for i = 1:size(u,2)
        for j = 1:n
            %(x, y)
            Nip = N(u(i), j-1, U, p);
            %res(i, 1) = u(i);
            res(i, 1) = res(i, 1) + P(j, 1) * Nip;
            res(i, 2) = res(i, 2) + P(j, 2) * Nip;
            res(i, 3) = res(i, 3) + P(j, 3) * Nip;
        end
    end
end


function  res = N(x, i, u, p)
    if p ==0
        if x>=u(i+1) && x<u(i+2)
            res = 1;
            return;
        end
        res = 0;
        return;
    end
    if u(i+p+1) == u(i+1) 
        a = 1;
    else 
        a =  (x - u(i+1))/(u(i+p+1) - u(i+1));
    end
    if u(i+p+2) == u(i+2)
        b = 1;
    else 
        b = (u(i+p+2) - x)/(u(i+p+2)-u(i+2));
    end
    res = a*N(x,i,u,p-1) + b*N(x,i+1,u,p-1);
end

%输出：各点u'
function res = myprogram(C, u, vmax, amax, iter_num)
    dCdu = get_first_gradient(C, u);
    dCdu2 = get_first_gradient(dCdu, u);
    dCdu3 = get_first_gradient(dCdu2, u);
    N = length(u);

    %目标函数
    func = @(dudt) sum(1./dudt);

    %线性不等式约束（速度约束）
    A = [];
    b = [];
    lb = zeros([N-1, 1]);
    ub = zeros([N-1, 1]);
    for i = 1:N-1
        ub(i) = vmax / norm(dCdu(i, :), 2);
    end

    %线性等式约束（初始速度/末速度）
%     Aeq = zeros([1, N-1]);
%     beq = zeros([2, 1]);
%     Aeq(1, 1) = norm(dCdu(1, :), 1); %初速度
%     Aeq(2, N-1) = norm(dCdu(N-1, :), 1); %末速度
%     beq(1) = v0;
%     beq(2) = vend;
    Aeq = []; %先不加初始末速度
    beq = [];
    %规划变量du/dt
    dudt0 = ones([N-1, 1]);
    
    options = optimoptions('fmincon', 'MaxFunctionEvaluations',iter_num);
    %非线性不等式约束(加速度约束，阶跃约束)
    dudt = fmincon(func, dudt0, A, b, Aeq, beq, lb, ub, @(dudt) non_linear_con(dudt, u, dCdu, dCdu2, dCdu3, amax), options);
    res = dudt;
end

%求dy/dx，采用后向差分，输出[m-1, :]
function res = get_first_gradient(y, x)
    [m, n] = size(y);
    res = zeros([m-1, n]);
    dx = x(2) - x(1);
    for i = 1:m-1
        res(i,:) = (y(i+1, :) - y(i, :)) ./ dx;
    end
end

%非均匀步长求导, 求dudt''
function res = get_dudt(dudt, u)
    M = length(u);
    res = zeros([M-2, 1]);
    for i = 1:M-2
        if dudt(i+1) == dudt(i)
            res(i) = 0;
        else
        res(i) = 2*(dudt(i+1) - dudt(i))*dudt(i)*dudt(i+1) / ((u(i+1)-u(i)) * dudt(i+1) + (u(i+2)-u(i+1)) * dudt(i));
        end
    end
end

function [c, ceq] = non_linear_con(dudt, u, dCdu, dCdu2, dCdu3, amax)
    %N个点
    N = length(dudt) + 1;
    ceq = [];
    c = zeros([N-2, 1]);
    
    %N-2个加速度约束
    dudt2 = get_dudt(dudt, u);
    for i = 1 : N-2
        c(i) = norm(dCdu2(i, :) .* dudt(i)^2 + dCdu(i, :) .* dudt2(i, :), 2) - amax;
    end

    %N-3个阶跃约束
    %dudt3 = get_dudt(dudt2, dudt);
    %for i = 1 : N-3
      %  c(i + N-2) = norm(dCdu3(i, :) .* dudt(i)^3 + dCdu2(i, :) .* (3 * dudt(i) * dudt2(i)) + dCdu .* dudt3(i), 2) - jmax;
    %end
end

function [t, v, a] = get_tva(dudt, C, u)
    N = length(u);
    t = zeros([N, 1]);
    v = zeros([N-1, 3]);
    a = zeros([N-2, 3]);
    for i = 2 : N
        t(i) = t(i-1) + (u(i) - u(i-1)) / dudt(i-1);
    end
    dCdu = get_first_gradient(C, u);
    for i = 1 : N-1
        v(i, :) = dCdu(i, :) .* dudt(i);
    end

    dudt2 = get_dudt(dudt, u);
    dCdu2 = get_first_gradient(dCdu, u);
    for i = 1 : N-2
        a(i, :) = dCdu2(i, :) .* dudt(i)^2 + dCdu(i, :) .* dudt2(i, :);
    end

end