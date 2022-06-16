%% SQP非线性优化器
%input@f：优化目标函数
%input@x0：待规划量初始值
%input@nonlcon：非线性约束 nonlcon = [c, ceq]
%output@x：优化得到的
function x = mySQP(f, x0, nonlcon)
%% 参数定义
[cin, ceq] = nonlcon(x0);
n1 = length(x0);  % 待规划变量个数
n2 = length(ceq); % 等号约束个数
n3 = length(cin);   % 不等号约束个数
n = n1 + n2 + n3;
epsilon = 1e-5;   % 微分精度
flag = true;
%% 迭代优化
while flag
    f(x0)
    c_set = logical(cin + 1e-3); % 确定Active set，g(x)>-1e-3的均加入有效集
    x_set = [true(n1 + n2, 1); c_set];
    % 获取 Hf
    Hf = zeros(n);
    for row = 1:n1
        for col = 1:n1
            Hf(row, col) = partial_f2(f, x0, row, col, epsilon);
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
        df(num) = -partial_f1(f, x0, num, epsilon);
    end
    df(n1 + 1:n) = -[ceq; cin];
    df = df(x_set);
    % 牛顿法获取迭代方向
    dx = (Hf + 1e-7*eye(length(Hf)))\df;
    dx = dx(1:n1);
    % 一维线搜索
    alpha = 1;
    while f(x0 + alpha*dx) > f(x0)
        alpha = alpha/1.5;
        if alpha < 1e-7
            break;
        end
    end
    x0 = x0 + alpha*dx;
    [cin, ceq] = nonlcon(x0);
    if norm(dx) < 1 && sum(logical(cin)) < 1
        flag = false;
    end
end
x = x0;
end

function df = partial_f1(f, x, index, epsilon)
x_tp = x; x_tp(index) = x_tp(index) + epsilon;
df = (f(x_tp) - f(x))/epsilon;
end

function ddf = partial_f2(f, x, index1, index2, epsilon)
x_tp = x; x_tp(index1) = x_tp(index1) + epsilon;
ddf = (partial_f1(f, x_tp, index2, epsilon) - partial_f1(f, x, index2, epsilon))/epsilon;
end

function dc = partial_c(c, x, index, epsilon)
x_tp = x; x_tp(index) = x_tp(index) + epsilon;
[cin, ceq] = c(x);
[cin_tp, ceq_tp] = c(x_tp);
dc = [ceq_tp - ceq; cin_tp - cin]/epsilon;
end