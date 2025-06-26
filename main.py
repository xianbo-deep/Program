import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False


# 系统辨识函数
def two_point_method(t, y):
    y_0 = y[0]
    y_ss = np.mean(y[-10:])
    y_39 = y_0 + 0.393 * (y_ss - y_0)
    y_63 = y_0 + 0.632 * (y_ss - y_0)

    t_39 = np.interp(y_39, y, t)
    t_63 = np.interp(y_63, y, t)

    tau = 2 * (t_63 - t_39)
    theta = 2 * t_39 - t_63

    u_step = volte[0]
    K = (y_ss - y_0) / u_step

    return K, tau, theta


# PID控制器类 - 简化和优化
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = 50  # 降低积分限制，减少积分饱和

    def update(self, current_value, dt):
        error = self.setpoint - current_value

        # 增加比例限制减少过冲
        proportional = self.Kp * error

        # 仅在误差范围内积分
        if abs(error) < 5:  # 只在误差小于5℃时积分
            self.integral += error * dt

        # 限制积分项
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return proportional + self.Ki * self.integral + self.Kd * derivative


# 简化的灰狼优化算法
class GreyWolfOptimizer:
    def __init__(self, sys_params, setpoint=35, wolves=5, iterations=25):
        self.K, self.tau, self.theta = sys_params
        self.setpoint = setpoint
        self.n_wolves = wolves
        self.max_iter = iterations

        # 参数搜索范围 - 减少Kd范围以降低超调
        self.bounds = np.array([
            [1.0, 50.0],  # Kp范围
            [0.01, 10.0],  # Ki范围 - 缩小
            [10.0, 200.0]  # Kd范围
        ])

        # 初始化狼群
        self.wolves = np.zeros((wolves, 3))
        self.fitness = np.full(wolves, 1e6)

        # 随机初始化位置
        for i in range(wolves):
            for j in range(3):
                self.wolves[i, j] = np.random.uniform(*self.bounds[j])

        # 记录最佳方案
        self.alpha = self.wolves[0].copy()
        self.alpha_fitness = 1e6
        self.fitness_history = []

    def evaluate(self, params):
        """评估PID参数的性能"""
        Kp, Ki, Kd = params

        # 仿真时间与步长
        sim_time = 30000
        points = 300  # 减少点数量加快速度
        time = np.linspace(0, sim_time, points)
        dt = time[1] - time[0]

        # 初始化
        temp = np.ones_like(time) * 20
        pid = PIDController(Kp, Ki, Kd, self.setpoint)

        # 跟踪最大超调量
        max_overshoot = 0

        for i in range(1, len(time)):
            # 延迟处理简化
            delay_idx = max(0, i - int(self.theta / dt)) if self.theta / dt > 0 else i
            current_temp = temp[delay_idx]

            u = pid.update(current_temp, dt)
            u = np.clip(u, 0, 10)  # 电压限制

            # 系统响应计算 - 简化和稳定化
            previous_temp = temp[i - 1] - 20
            new_temp_value = previous_temp + (self.K * u - previous_temp) * dt / self.tau

            # 检查超调
            current_temp_val = new_temp_value + 20
            if current_temp_val > self.setpoint:
                overshoot = current_temp_val - self.setpoint
                if overshoot > max_overshoot:
                    max_overshoot = overshoot

            # 更新温度
            temp[i] = np.clip(new_temp_value + 20, 0, 100)

        # 性能指标 - 增加对超调的惩罚权重
        error = np.abs(temp - self.setpoint)
        itae = np.sum(error * time)

        # 大幅增加超调惩罚权重
        if max_overshoot > 0.1:
            itae += max_overshoot * 2000
        elif max_overshoot > 0.05:
            itae += max_overshoot * 1000

        return itae

    def optimize(self):
        print("开始灰狼优化...")

        # 计算初始适应度
        best_score = float('inf')
        for i in range(self.n_wolves):
            self.fitness[i] = self.evaluate(self.wolves[i])
            if self.fitness[i] < best_score:
                best_score = self.fitness[i]
                self.alpha = self.wolves[i].copy()
                self.alpha_fitness = best_score

        print(f"初始最佳适应度: {self.alpha_fitness:.2f}")
        self.fitness_history.append(self.alpha_fitness)

        # 主优化循环简化
        for it in range(self.max_iter):
            a = 2 - it * (2 / self.max_iter)  # 线性递减系数

            for i in range(self.n_wolves):
                r1, r2 = np.random.rand(3), np.random.rand(3)
                A = 2 * a * r1 - a
                C = 2 * r2

                # 向Alpha移动
                for j in range(3):
                    D = np.abs(C[j] * self.alpha[j] - self.wolves[i, j])
                    self.wolves[i, j] = np.clip(self.alpha[j] - A[j] * D,
                                                self.bounds[j][0],
                                                self.bounds[j][1])

                # 评估新位置
                new_fitness = self.evaluate(self.wolves[i])

                # 更新Alpha
                if new_fitness < self.alpha_fitness:
                    self.alpha = self.wolves[i].copy()
                    self.alpha_fitness = new_fitness
                    print(f"迭代 {it + 1}: 新Alpha, ITAE={new_fitness:.2f}")

            # 记录最佳适应度
            self.fitness_history.append(self.alpha_fitness)

        print("优化完成!")
        return self.alpha


# 结果分析和绘图函数简化
def simulate_and_plot(K, tau, theta, best_params):
    sim_time = 30000
    points = 300
    time = np.linspace(0, sim_time, points)
    temp = np.ones(points) * 20
    control = np.zeros(points)
    dt = time[1] - time[0]

    pid = PIDController(best_params[0], best_params[1], best_params[2], 35)

    # 跟踪超调量
    max_overshoot = 0

    for i in range(1, len(time)):
        # 无延迟简化
        current_temp = temp[i - 1]
        u = pid.update(current_temp, dt)
        u = np.clip(u, 0, 10)
        control[i] = u

        # 系统响应
        previous_temp = temp[i - 1] - 20
        new_temp_value = previous_temp + (K * u - previous_temp) * dt / tau
        temp[i] = np.clip(new_temp_value + 20, 0, 100)

        # 更新最大超调
        if temp[i] > 35:
            overshoot = temp[i] - 35
            if overshoot > max_overshoot:
                max_overshoot = overshoot

    # 绘图
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(time, temp, 'b-', label='温度')
    plt.axhline(35, color='r', linestyle='--', label='目标温度')
    plt.title(f'温度控制结果 (超调={max_overshoot:.2f}℃)')
    plt.ylabel('温度 (℃)')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(time, control, 'g-', label='控制信号')
    plt.xlabel('时间 (秒)')
    plt.ylabel('电压 (V)')
    plt.ylim(-0.5, 10.5)
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('control_result.png')
    plt.show()

    # 性能分析
    error = temp - 35
    itae = np.sum(np.abs(error) * time)
    steady_error = np.mean(np.abs(error[-50:]))  # 后50点作为稳态误差

    print("\n控制性能:")
    print(f"最大超调: {max_overshoot:.2f}℃")
    print(f"稳态误差: {steady_error:.3f}℃")
    print(f"ITAE指标: {itae:.1f}")

    # 保存结果
    results = pd.DataFrame({'time': time, 'temperature': temp, 'voltage': control})
    results.to_csv('control_results.csv', index=False)


if __name__ == '__main__':
    try:
        # 加载数据
        data = pd.read_csv('./B 任务数据集.csv')
        time_data = data['time'].values
        temperature_data = data['temperature'].values
        global volte
        volte = data['volte'].values

        # 系统辨识
        K, tau, theta = two_point_method(time_data, temperature_data)
        print(f"辨识参数: K={K:.3f}, tau={tau:.3f}, theta={theta:.3f}")

        # 使用灰狼算法优化PID参数
        gwo = GreyWolfOptimizer((K, tau, theta))
        best_params = gwo.optimize()
        print(f"\n最优PID参数: Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}")

        # 可视化优化过程
        plt.figure(figsize=(10, 5))
        plt.plot(gwo.fitness_history, 'bo-')
        plt.title('优化过程收敛曲线')
        plt.xlabel('迭代次数')
        plt.ylabel('适应度 (ITAE)')
        plt.grid(True)
        plt.savefig('convergence.png')
        plt.show()

        # 仿真和绘制结果
        simulate_and_plot(K, tau, theta, best_params)
        print("结果已保存到 control_results.csv 和 control_result.png")

    except Exception as e:
        print(f"错误: {e}")