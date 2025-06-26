import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, lsim
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


# PID控制器类
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


# 灰狼优化算法
class GreyWolfOptimizer:
    def __init__(self, sys_params, setpoint=35, wolves=7, iterations=30):
        """
        sys_params: 系统参数 (K, tau, theta)
        wolves: 狼群数量
        iterations: 最大迭代次数
        """
        self.K, self.tau, self.theta = sys_params
        self.setpoint = setpoint
        self.n_wolves = wolves
        self.max_iter = iterations

        # 参数搜索范围
        self.bounds = np.array([
            [0.1, 50.0],  # Kp范围
            [100, 3000],  # Ki范围
            [1.0,100.0]  # Kd范围
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
        self.beta = self.wolves[1].copy()
        self.delta = self.wolves[2].copy()
        self.alpha_fitness = 1e6
        self.fitness_history = []

    def evaluate(self, params):
        """评估PID参数的性能（ITAE指标）"""
        Kp, Ki, Kd = params

        # 仿真PID控制
        time = np.linspace(0, 7200,7201)  # 150秒仿真，0.5秒步长
        temp = np.ones_like(time) * 20  # 初始温度
        control = np.zeros_like(time)
        dt = time[1] - time[0]

        pid = PIDController(Kp, Ki, Kd, self.setpoint)

        for i in range(1, len(time)):
            # 考虑测量延迟
            delay_idx = max(0, i - int(self.theta / dt))
            current_temp = temp[delay_idx]

            u = pid.update(current_temp, dt)
            u = np.clip(u, 0, 10)  # 电压限制0-10V
            control[i] = u

            # 系统响应
            sys = lti([self.K], [self.tau, 1])
            _, y, _ = lsim(sys, U=[u, u], T=[0, dt], X0=temp[i - 1] - 20)
            temp[i] = y[-1] + 20

            # 防止溢出
            temp[i] = np.clip(temp[i], 0, 100)

        # 计算ITAE指标（时间加权绝对误差）
        error = np.abs(temp - self.setpoint)
        itae = np.sum(error * time)

        # 添加超调惩罚
        overshoot = max(0, np.max(temp) - self.setpoint)
        if overshoot > 0.1:  # 如果超调超过0.1℃，增加惩罚
            itae += overshoot * 50

        return itae

    def optimize(self):
        """执行灰狼优化"""
        print("开始灰狼优化...")

        # 计算初始适应度
        for i in range(self.n_wolves):
            self.fitness[i] = self.evaluate(self.wolves[i])

        # 主循环
        for it in range(self.max_iter):
            # 更新衰减系数a
            a = 2 - it * (2 / self.max_iter)

            # 更新每只狼的位置
            for i in range(self.n_wolves):
                # Alpha, Beta, Delta引导
                for leader in [self.alpha, self.beta, self.delta]:
                    r1, r2 = np.random.rand(3), np.random.rand(3)
                    A = 2 * a * r1 - a  # 计算A
                    C = 2 * r2  # 计算C

                    # 根据领导者的位置更新当前位置
                    for j in range(3):
                        D = np.abs(C[j] * leader[j] - self.wolves[i, j])
                        self.wolves[i, j] = np.clip(
                            leader[j] - A[j] * D,
                            self.bounds[j][0],
                            self.bounds[j][1]
                        )

                # 评估新位置
                new_fitness = self.evaluate(self.wolves[i])

                # 更新Alpha, Beta, Delta
                if new_fitness < self.fitness[i]:
                    self.fitness[i] = new_fitness

                    # 更新领导者
                    if new_fitness < self.alpha_fitness:
                        self.delta = self.beta.copy()
                        self.beta = self.alpha.copy()
                        self.alpha = self.wolves[i].copy()
                        self.alpha_fitness = new_fitness
                    elif new_fitness < self.evaluate(self.beta):
                        self.delta = self.beta.copy()
                        self.beta = self.wolves[i].copy()
                    elif new_fitness < self.evaluate(self.delta):
                        self.delta = self.wolves[i].copy()

            # 记录最佳适应度
            self.fitness_history.append(self.alpha_fitness)
            print(f"迭代 {it + 1}/{self.max_iter} | 最佳ITAE: {self.alpha_fitness:.2f}")

        print("优化完成!")
        return self.alpha

    def plot_convergence(self):
        """绘制收敛曲线"""
        plt.figure(figsize=(10, 5))
        plt.plot(self.fitness_history, 'b-o')
        plt.title('灰狼优化收敛过程')
        plt.xlabel('迭代次数')
        plt.ylabel('ITAE指标')
        plt.grid(True)
        plt.show()


# 结果分析函数
def analyze_control(time, temperature, setpoint=35):
    """分析控制性能"""
    error = temperature - setpoint
    steady_mask = (time > time[-1] * 0.8)  # 后20%数据为稳态区
    print(temperature[-100:])
    # 计算关键指标
    rise_time = time[np.where(temperature >= 0.9 * setpoint)[0][0]]
    settling_time = time[np.where(np.abs(error) < 0.5)[0][0]]
    overshoot = max(0, np.max(temperature) - setpoint)
    steady_error = np.mean(np.abs(error[steady_mask]))
    itae = np.sum(np.abs(error) * time)

    print("\n控制性能分析:")
    print(f"上升时间: {rise_time:.1f}s")
    print(f"稳定时间: {settling_time:.1f}s")
    print(f"超调量: {overshoot:.2f}℃")
    print(f"稳态误差: {steady_error:.3f}℃")
    print(f"ITAE指标: {itae:.1f}")

    # 绘制结果
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(time, temperature, 'b-', linewidth=2)
    plt.axhline(setpoint, color='r', linestyle='--', label='目标温度')
    plt.fill_between(time, setpoint - 0.5, setpoint + 0.5, color='gray', alpha=0.2)
    plt.ylabel('温度 (℃)')
    plt.title(f'加热炉温度控制 (Kp={best_params[0]:.3f}, Ki={best_params[1]:.3f}, Kd={best_params[2]:.3f})')
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time, control_signal, 'g-', linewidth=1.5)
    plt.xlabel('时间 (秒)')
    plt.ylabel('控制电压 (V)')
    plt.title('控制信号变化')
    plt.ylim(-0.5, 10.5)
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    return {
        'rise_time': rise_time,
        'settling_time': settling_time,
        'overshoot': overshoot,
        'steady_error': steady_error,
        'itae': itae
    }


if __name__ == '__main__':
    # 加载加热炉数据
    data = pd.read_csv('./B 任务数据集.csv')
    time_data = data['time'].values
    temperature_data = data['temperature'].values
    global volte
    volte = data['volte'].values

    # 系统辨识
    # K, tau, theta = two_point_method(time_data, temperature_data)
    # print(f"辨识参数: K={K:.3f}, tau={tau:.3f}, theta={theta:.3f}")

    # 使用灰狼算法优化PID参数
    gwo = GreyWolfOptimizer(
        sys_params=(K, tau, theta),
        setpoint=35,
        wolves=7,
        iterations=5
    )
    # best_params = gwo.optimize()
    # print(f"\n最优PID参数: Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}")

    # 绘制优化过程
    # gwo.plot_convergence()

    # 仿真最佳参数控制效果
    print("\n开始仿真最优控制效果...")
    sim_time = np.linspace(0, 20000, 20001)
    sim_temp = np.ones_like(sim_time) * 20
    control_signal = np.zeros_like(sim_time)
    dt = sim_time[1] - sim_time[0]

    # pid = PIDController(best_params[0],best_params[1], best_params[2])
    pid = PIDController(2, 500, 10,35)
    for i in range(1, len(sim_time)):
        # 考虑测量延迟
        delay_idx = max(0, i - int(theta / dt))
        current_temp = sim_temp[delay_idx]

        u = pid.update(current_temp, dt)
        u = np.clip(u, 0, 10)  # 限制电压范围
        control_signal[i] = u

        # 系统响应
        sys = lti([K], [tau, 1])
        _, y, _ = lsim(sys, U=[u, u], T=[0, dt], X0=sim_temp[i - 1] - 20)
        sim_temp[i] = y[-1] + 20

    # 分析控制性能
    perf = analyze_control(sim_time, sim_temp, 35)

    # 保存优化结果
    results = pd.DataFrame({
        'time': sim_time,
        'temperature': sim_temp,
        'voltage': control_signal
    })
    results.to_csv('heating_furnace_control_results.csv', index=False)
    print("控制结果已保存到 heating_furnace_control_results.csv")