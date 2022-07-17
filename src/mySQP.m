%% SQP非线性优化器
%input@f：优化目标函数
%input@x0：待规划量初始值
%input@nonlcon：非线性约束 nonlcon = [c, ceq]
%output@x：求解得到函数最小值点
function x = mySQP(f, x0, nonlcon)
%% 参数定义
[cin, ceq] = nonlcon(x0);
n1 = length(x0);  % 待规划变量个数
n2 = length(ceq); % 等号约束个数
n3 = length(cin);   % 不等号约束个数
n = n1 + n2 + n3;
epsilon = 1e-5;   % 微分精度
lam = ones(n2 + n3, 1)*1e2; % 拉格朗日乘子
flag = true;
%% 迭代优化
while flag
    f(x0)
    c_set = [true(n2, 1); logical(cin + 1e-3)]; % 确定Active set，g(x)>-1e-3的均加入有效集
%     sum(c_set) - n2
    x_set = [true(n1, 1); c_set];
    % 获取 Hf
    Hf = zeros(n);
    for row = 1:n1
        for col = 1:n1
            Hf(row, col) = partial_L2(f, nonlcon, x0, lam, c_set, row, col, epsilon);
        end
    end
    for num = 1:n1
        Hf(n1 + 1:n, num) = partial_c(nonlcon, x0, num, epsilon);
    end
    Hf(1:n1, n1 + 1:n) = Hf(n1 + 1:n, 1:n1).';
    Hf = reshape(Hf(logical(x_set*x_set.')), [sum(x_set), sum(x_set)]);
    % 获取 df
    df = zeros(n, 1);
    for num = 1:n1
        df(num) = -partial_L1(f, nonlcon, x0, lam, c_set, num, epsilon);
    end
    df(n1 + 1:n) = -[ceq; cin];
    df = df(x_set);
    % 牛顿法获取迭代方向
    dx = (Hf + 1e-3*eye(length(df)))\df;
    dlam = dx(n1 + 1:end);
    dx = dx(1:n1);
    % 一维线搜索
    alpha = 1;
    while f(x0 + alpha*dx) > f(x0)
        alpha = alpha/2;
        if alpha < 1e-5
            break;
        end
    end
    x0 = x0 + alpha*dx;
    lam(c_set) = lam(c_set) + alpha*dlam;
    [cin, ceq] = nonlcon(x0);
    if norm(dx) < 1 && sum(logical(cin)) < 1
        flag = false;
    end
end
x = x0;
end

function L = getL(f, c, x, lam, c_set)
[cin, ceq] = c(x);
L = f(x) + dot(c_set, lam.*abs([ceq; cin]));
end

function dL = partial_L1(f, c, x, lam, c_set, index, epsilon)
x_tp = x; x_tp(index) = x_tp(index) + epsilon;
dL = (getL(f, c, x_tp, lam, c_set) - getL(f, c, x, lam, c_set))/epsilon;
end

function HL = partial_L2(f, c, x, lam, c_set, index1, index2, epsilon)
x_tp = x; x_tp(index1) = x_tp(index1) + epsilon;
HL = (partial_L1(f, c, x_tp, lam, c_set, index2, epsilon) - partial_L1(f, c, x, lam, c_set, index2, epsilon))/epsilon;
end

function dc = partial_c(c, x, index, epsilon)
x_tp = x; x_tp(index) = x_tp(index) + epsilon;
[cin, ceq] = c(x);
[cin_tp, ceq_tp] = c(x_tp);
dc = [ceq_tp - ceq; cin_tp - cin]/epsilon;
end