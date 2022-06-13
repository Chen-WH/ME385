# ME385

- kine_test.m：测试程序/函数使用示例
- myfkine_poe.m：正向运动学，input@theta：指数坐标法各旋量关节角度[6, N]，output@gst：末端位置齐次变换矩阵形式[4, 4, N]
- myikine_poe.m：逆向运动学，input@gst_set：末端位置齐次变换矩阵形式[4, 4, n]，input@theta0：初始位置关节空间[6, 1]，output@theta_set：指数坐标法各旋量关节角度[6, n]为解的个数
- myikine_poe8.m：逆向运动学（生成所有可能逆解），input@gst：末端位置齐次变换矩阵形式[4, 4]，output@theta：指数坐标法各旋量关节角度[6, m]，m(<=8)为解的个数