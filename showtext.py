############################################## 四元数输出 ###############################################################
from fastapi import FastAPI
import uvicorn

app = FastAPI()

# 定义四元数全局变量
local_quat = [1.0, 0.0, 0.0, 0.0]

@app.get("/quaternion")
def get_quaternion():
    return str(local_quat)



############################################ 卡尔曼滤波 #####################################################################
import numpy as np

class KalmanFilter:
    def __init__(self, F, B, H, Q, R, x0, P0):  #复制以方便重新初始化
        self.F = np.copy(F)  # 状态转移矩阵（复制）
        self.B = np.copy(B)  # 控制输入矩阵（复制）
        self.H = np.copy(H)  # 观测矩阵（复制）
        self.Q = np.copy(Q)  # 过程噪声协方差（复制）
        self.R = np.copy(R)  # 观测噪声协方差（复制）
        self.x0 = np.copy(x0)  # 初始状态估计（复制）
        self.P0 = np.copy(P0)  # 初始协方差估计（复制）
        self.x = np.copy(x0)  # 当前状态估计（复制）
        self.P = np.copy(P0)  # 当前协方差估计（复制）

    def predict(self, u):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)
        return self.x

# 示例参数
dt = 1.0  # 时间步长
F = np.array([[1, 0, 0, dt, 0, 0],
              [0, 1, 0, 0, dt, 0],
              [0, 0, 1, 0, 0, dt],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])  # 状态转移矩阵

B = np.array([[0.5*dt**2, 0, 0],
              [0, 0.5*dt**2, 0],
              [0, 0, 0.5*dt**2],
              [dt, 0, 0],
              [0, dt, 0],
              [0, 0, dt]])  # 控制输入矩阵

H = np.eye(6)  # 观测矩阵
Q = np.eye(6) * 0.01  # 过程噪声协方差
R = np.eye(6) * 0.1  # 观测噪声协方差
x0 = np.zeros(6)  # 初始状态估计
P0 = np.eye(6)  # 初始协方差估计

kf = KalmanFilter(F, B, H, Q, R, x0, P0)






import serial
import tkinter as tk
from tkinter import ttk
import threading
import queue
import math
from tkinter import font
import numpy as np
import time

# 配置串口
port = 'COM13'  # 串口号
baudrate = 9600  # 波特率
timeout = 1  # 超时时间


# 初始化本地四元数
# local_quat = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
#本地绝对加速度
local_absolute_acc = [0.0, 0.0, 0.0]
# 正在录制重力加速度
recording_gravity = False
recording_start_time = 0
recording_duration = 10
acc_data = []
quat_data = []

# 全局变量存储标定的重力加速度
calibrated_gravity = np.array([0.0, 0.0, 0.0])
# 积分速度和位移
integral_velocity = np.array([0.0, 0.0, 0.0])
integral_position = np.array([0.0, 0.0, 0.0])

def recording_gravity_control_thread():
    global recording_start_time, recording_gravity
    recording_start_time = time.time()
    recording_gravity = True


def calibrate_gravity():
    global calibrated_gravity
    
    # 将加速度从传感器坐标系转换到绝对坐标系
    absolute_acc = []
    for acc, quat in zip(acc_data, quat_data):
        # 计算旋转矩阵
        w, x, y, z = quat
        rotation_matrix = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
        
        # 将加速度转换到绝对坐标系
        acc_absolute = np.dot(rotation_matrix, acc)
        absolute_acc.append(acc_absolute)
    
    # 计算绝对加速度的均值和方差
    absolute_acc = np.array(absolute_acc)
    mean_acc = np.mean(absolute_acc, axis=0)
    # var_acc = np.var(absolute_acc, axis=0)
    var_acc = np.std(absolute_acc, axis=0)
    
    print(f"绝对加速度均值: {mean_acc}")
    print(f"绝对加速度方差: {var_acc}")
    
    # 剔除三倍方差之外的异常值
    mask = np.all(np.abs(absolute_acc - mean_acc) < 3 * np.sqrt(var_acc), axis=1)
    filtered_acc = absolute_acc[mask]
    
    # 重新计算均值和方差
    mean_acc_filtered = np.mean(filtered_acc, axis=0)
    # var_acc_filtered = np.var(filtered_acc, axis=0)
    var_acc_filtered = np.std(filtered_acc, axis=0)
    
    print(f"过滤后的绝对加速度均值: {mean_acc_filtered}")
    print(f"过滤后的绝对加速度方差: {var_acc_filtered}")
    
    # 设置标定的重力加速度
    calibrated_gravity = mean_acc_filtered
    print(f"标定的重力加速度: {calibrated_gravity}")

# 四阶龙格-库塔法更新四元数
def runge_kutta_4th_order(quat, gyro, dt):
    w, x, y, z = quat
    wx, wy, wz = gyro

    # 四阶龙格-库塔法的四个步骤
    k1_w = (-x * wx - y * wy - z * wz) * 0.5
    k1_x = (w * wx + y * wz - z * wy) * 0.5
    k1_y = (w * wy - x * wz + z * wx) * 0.5
    k1_z = (w * wz + x * wy - y * wx) * 0.5

    k2_w = (-(x + 0.5 * k1_x * dt) * wx - (y + 0.5 * k1_y * dt) * wy - (z + 0.5 * k1_z * dt) * wz) * 0.5
    k2_x = ((w + 0.5 * k1_w * dt) * wx + (y + 0.5 * k1_y * dt) * wz - (z + 0.5 * k1_z * dt) * wy) * 0.5
    k2_y = ((w + 0.5 * k1_w * dt) * wy - (x + 0.5 * k1_x * dt) * wz + (z + 0.5 * k1_z * dt) * wx) * 0.5
    k2_z = ((w + 0.5 * k1_w * dt) * wz + (x + 0.5 * k1_x * dt) * wy - (y + 0.5 * k1_y * dt) * wx) * 0.5

    k3_w = (-(x + 0.5 * k2_x * dt) * wx - (y + 0.5 * k2_y * dt) * wy - (z + 0.5 * k2_z * dt) * wz) * 0.5
    k3_x = ((w + 0.5 * k2_w * dt) * wx + (y + 0.5 * k2_y * dt) * wz - (z + 0.5 * k2_z * dt) * wy) * 0.5
    k3_y = ((w + 0.5 * k2_w * dt) * wy - (x + 0.5 * k2_x * dt) * wz + (z + 0.5 * k2_z * dt) * wx) * 0.5
    k3_z = ((w + 0.5 * k2_w * dt) * wz + (x + 0.5 * k2_x * dt) * wy - (y + 0.5 * k2_y * dt) * wx) * 0.5

    k4_w = (-(x + k3_x * dt) * wx - (y + k3_y * dt) * wy - (z + k3_z * dt) * wz) * 0.5
    k4_x = ((w + k3_w * dt) * wx + (y + k3_y * dt) * wz - (z + k3_z * dt) * wy) * 0.5
    k4_y = ((w + k3_w * dt) * wy - (x + k3_x * dt) * wz + (z + k3_z * dt) * wx) * 0.5
    k4_z = ((w + k3_w * dt) * wz + (x + k3_x * dt) * wy - (y + k3_y * dt) * wx) * 0.5

    # 更新四元数
    w += (k1_w + 2 * k2_w + 2 * k3_w + k4_w) * dt / 6
    x += (k1_x + 2 * k2_x + 2 * k3_x + k4_x) * dt / 6
    y += (k1_y + 2 * k2_y + 2 * k3_y + k4_y) * dt / 6
    z += (k1_z + 2 * k2_z + 2 * k3_z + k4_z) * dt / 6

    # 归一化四元数
    norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    return [w, x, y, z]

def rk4_integration(integral_velocity, integral_position, local_absolute_acc, dt):
    # RK4 method for integrating acceleration to velocity and velocity to position
    def derivative(v, a):
        return a  # dv/dt = a

    # RK4 steps for velocity
    k1_v = derivative(integral_velocity, local_absolute_acc)
    k2_v = derivative(integral_velocity + 0.5 * dt * k1_v, local_absolute_acc)
    k3_v = derivative(integral_velocity + 0.5 * dt * k2_v, local_absolute_acc)
    k4_v = derivative(integral_velocity + dt * k3_v, local_absolute_acc)

    integral_velocity += (dt / 6.0) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)

    # RK4 steps for position
    k1_p = derivative(integral_position, integral_velocity)
    k2_p = derivative(integral_position + 0.5 * dt * k1_p, integral_velocity)
    k3_p = derivative(integral_position + 0.5 * dt * k2_p, integral_velocity)
    k4_p = derivative(integral_position + dt * k3_p, integral_velocity)

    integral_position += (dt / 6.0) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p)

    return integral_velocity, integral_position


# 打开串口
ser = serial.Serial(port, baudrate, timeout=timeout)

# 四元数转欧拉角
def quaternion_to_euler(w, x, y, z):
    # 计算欧拉角（roll, pitch, yaw）
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    # 将弧度转换为角度
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    return roll, pitch, yaw


# 创建主窗口
root = tk.Tk()
root.title("IMU 数据监控")
root.configure(bg='black')

# 定义颜色
colors = {'X': 'red', 'Y': 'green', 'Z': 'blue'}
axis_labels = {'acc': '加速度', 'gyro': '角速度', 'quat': '四元数', 'euler': '欧拉角', 'Temperature': '温度', 'local_quat': '本地四元数',
                'local_euler': '本地欧拉角', 'absolute_acc': '本地绝对加速度',
                'local_position': '本地位移', 'local_velocity': '本地速度'}

# 设置字体
custom_font = font.Font(family=("Consolas"), size=18)

# 创建标签和文本框
labels = {}
entries = {}

# 通用函数：创建轴标签和文本框
def create_axis_widgets(parent, prefix, row_start, axes, is_quat=False):
    # 标题放在最左边
    labels[prefix] = tk.Label(root, text=axis_labels[prefix], fg="white", bg="black", font=custom_font)
    labels[prefix].grid(row=row_start, column=0, padx=10, pady=5, sticky="w")

    # 每个轴的表头和文本框放在同一行
    for i, axis in enumerate(axes):
        # 表头
        labels[f'{prefix}{axis}'] = tk.Label(root, text=f"{axis}:", fg=colors.get(axis, 'white'), bg="black", font=custom_font)
        labels[f'{prefix}{axis}'].grid(row=row_start, column=i + 1, padx=(10, 0), pady=5, sticky="e")

        # 文本框
        entries[f'{prefix}{axis}'] = tk.Entry(root, width=15, bg="black", fg=colors.get(axis, 'white'), font=custom_font, insertbackground='white')
        entries[f'{prefix}{axis}'].grid(row=row_start, column=i + 2, padx=(0, 80), pady=5, sticky="w")

def mcu_ser_quat_zero():
    #串口发送zeroposition
    ser.write(b'zeroposition;\n')
    print('zeroposition;')

# 加速度
create_axis_widgets(root, 'acc', 0, ['X', 'Y', 'Z'])

# 角速度
create_axis_widgets(root, 'gyro', 1, ['X', 'Y', 'Z'])

# 四元数
create_axis_widgets(root, 'quat', 2, ['W', 'X', 'Y', 'Z'], is_quat=True)

# 欧拉角
create_axis_widgets(root, 'euler', 3, ['Roll', 'Pitch', 'Yaw'])

# mcu欧拉角归零
mcu_quat_zero_button = tk.Button(root, text="参考姿态", command=mcu_ser_quat_zero, bg="black", fg="white", font=custom_font)
mcu_quat_zero_button.grid(row=3, column=5, padx=0, pady=5)

# 温度
create_axis_widgets(root, 'Temperature', 4, [''])

# 在GUI中增加一行显示本地四元数
create_axis_widgets(root, 'local_quat', 5, ['W', 'X', 'Y', 'Z'], is_quat=True)

# 在GUI中增加一行显示本地欧拉角
create_axis_widgets(root, 'local_euler', 6, ['Roll', 'Pitch', 'Yaw'])

# 本地绝对加速度
create_axis_widgets(root, 'absolute_acc', 7, ['X', 'Y', 'Z'])

# 在加速度行右边添加“重力标定”按钮
calibrate_button = tk.Button(root, text="重力标定", command=recording_gravity_control_thread, bg="black", fg="white", font=custom_font)
calibrate_button.grid(row=7, column=5, padx=0, pady=5)

#本地位移和速度
create_axis_widgets(root, 'local_velocity', 8, ['X', 'Y', 'Z'])
create_axis_widgets(root, 'local_position', 9, ['X', 'Y', 'Z'])

# 数据队列，用于线程间通信
data_queue = queue.Queue()

# 串口读取线程
def serial_read_thread():
    global local_quat, local_absolute_acc, integral_velocity, integral_position, kf
    while True:
        try:
            # 读取串口数据
            data = ser.readline()
            if data:
                # 将数据解码并拆分为列表
                decoded_data = data.decode('utf-8').strip()
                data_list = decoded_data.split(',')  # 假设数据以逗号分隔
                #转浮点数
                data_list = list(map(float, data_list))

                #更新本地四元数
                gyro = [data_list[3], data_list[4], data_list[5]]
                #修正恒定误差
                gyro[0] -= 0.001413
                gyro[1] -= -0.000219
                gyro[2] -= 0.000515
                # die_distance = 0.0022 #量程2000时数字分辨率0.00107弧度/s，大多漂移一般在0-0.0022，保险的数字0.005已经几乎没有零飘。建议范围0.33-0.0055
                die_distance = 0.0055
                if gyro[0] > -die_distance and gyro[0] < die_distance:
                    gyro[0] = 0
                if gyro[1] > -die_distance and gyro[1] < die_distance:
                    gyro[1] = 0
                if gyro[2] > -die_distance and gyro[2] < die_distance:
                    gyro[2] = 0
                local_quat = runge_kutta_4th_order(local_quat, gyro, 0.005)

                if recording_gravity and time.time() - recording_start_time <= recording_duration - 0.1:
                    acc_data.append([data_list[0], data_list[1], data_list[2]])
                    quat_data.append(local_quat)
                    kf = None
                if recording_gravity == False and kf != None:
                    w, x, y, z = local_quat
                    rotation_matrix = np.array([
                        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
                    ])
                    local_absolute_acc = np.dot(rotation_matrix, np.array([data_list[0], data_list[1], data_list[2]])) - calibrated_gravity

                    #暂时设置0.03的死区
                    # die_distance_local_acc = 0.1
                    # if local_absolute_acc[0] > -die_distance_local_acc and local_absolute_acc[0] < die_distance_local_acc:
                    #     local_absolute_acc[0] = 0
                    # if local_absolute_acc[1] > -die_distance_local_acc and local_absolute_acc[1] < die_distance_local_acc:
                    #     local_absolute_acc[1] = 0
                    # if local_absolute_acc[2] > -die_distance_local_acc and local_absolute_acc[2] < die_distance_local_acc:
                    #     local_absolute_acc[2] = 0

                    #直接积分法计算位置和速度
                    # integral_velocity += local_absolute_acc * 0.005
                    # integral_position += integral_velocity * 0.005
                    #使用RK4积分法计算位置和速度
                    # rk4_integration(integral_velocity, integral_position, local_absolute_acc, 0.005)
                    #使用卡尔曼滤波计算位置和速度
                    u = local_absolute_acc
                    z = [local_absolute_acc[0], local_absolute_acc[1], local_absolute_acc[2], gyro[0], gyro[1], gyro[2]]
                    predicted_state = kf.predict(u)
                    filtered_state = kf.update(z)
                    integral_velocity = filtered_state[:3]
                    integral_position = filtered_state[3:]
                    

                # 将数据放入队列
                data_queue.put(data_list)
        except Exception as e:
            print(f"串口读取错误: {e}")
            break

# 启动串口读取线程
thread = threading.Thread(target=serial_read_thread, daemon=True)
thread.start()

# 更新GUI数据
def update_gui():
    global recording_gravity, integral_velocity, integral_position, kf
    try:  
        # 从队列中获取最新数据
        while not data_queue.empty():
            data_list = data_queue.get_nowait()

            # 更新文本框内容
            entries['accX'].delete(0, tk.END)
            entries['accX'].insert(0, f"{float(data_list[0]):.5f}")
            entries['accY'].delete(0, tk.END)
            entries['accY'].insert(0, f"{float(data_list[1]):.5f}")
            entries['accZ'].delete(0, tk.END)
            entries['accZ'].insert(0, f"{float(data_list[2]):.5f}")

            entries['Temperature'].delete(0, tk.END)
            entries['Temperature'].insert(0, f"{float(data_list[6]):.5f}")

            entries['gyroX'].delete(0, tk.END)
            entries['gyroX'].insert(0, f"{float(data_list[3]):.5f}")
            entries['gyroY'].delete(0, tk.END)
            entries['gyroY'].insert(0, f"{float(data_list[4]):.5f}")
            entries['gyroZ'].delete(0, tk.END)
            entries['gyroZ'].insert(0, f"{float(data_list[5]):.5f}")

            entries['quatW'].delete(0, tk.END)
            entries['quatW'].insert(0, f"{data_list[7]:.4f}")
            entries['quatX'].delete(0, tk.END)
            entries['quatX'].insert(0, f"{data_list[8]:.4f}")
            entries['quatY'].delete(0, tk.END)
            entries['quatY'].insert(0, f"{data_list[9]:.4f}")
            entries['quatZ'].delete(0, tk.END)
            entries['quatZ'].insert(0, f"{data_list[10]:.4f}")

            # 计算欧拉角
            roll, pitch, yaw = quaternion_to_euler(
                float(data_list[7]), float(data_list[8]),
                float(data_list[9]), float(data_list[10])
            )
            entries['eulerRoll'].delete(0, tk.END)
            entries['eulerRoll'].insert(0, f"{roll:.5f}")
            entries['eulerPitch'].delete(0, tk.END)
            entries['eulerPitch'].insert(0, f"{pitch:.5f}")
            entries['eulerYaw'].delete(0, tk.END)
            entries['eulerYaw'].insert(0, f"{yaw:.5f}")

            # 显示本地四元数
            entries['local_quatW'].delete(0, tk.END)
            entries['local_quatW'].insert(0, f"{local_quat[0]:.4f}")
            entries['local_quatX'].delete(0, tk.END)
            entries['local_quatX'].insert(0, f"{local_quat[1]:.4f}")
            entries['local_quatY'].delete(0, tk.END)
            entries['local_quatY'].insert(0, f"{local_quat[2]:.4f}")
            entries['local_quatZ'].delete(0, tk.END)
            entries['local_quatZ'].insert(0, f"{local_quat[3]:.4f}")

            #计算本地欧拉角
            roll, pitch, yaw = quaternion_to_euler(
                local_quat[0], local_quat[1],
                local_quat[2], local_quat[3]
            )
            entries['local_eulerRoll'].delete(0, tk.END)
            entries['local_eulerRoll'].insert(0, f"{roll:.5f}")
            entries['local_eulerPitch'].delete(0, tk.END)
            entries['local_eulerPitch'].insert(0, f"{pitch:.5f}")
            entries['local_eulerYaw'].delete(0, tk.END)
            entries['local_eulerYaw'].insert(0, f"{yaw:.5f}")

            #显示本地绝对加速度
            entries['absolute_accX'].delete(0, tk.END)
            entries['absolute_accX'].insert(0, f"{local_absolute_acc[0]:.4f}")
            entries['absolute_accY'].delete(0, tk.END)
            entries['absolute_accY'].insert(0, f"{local_absolute_acc[1]:.4f}")
            entries['absolute_accZ'].delete(0, tk.END)
            entries['absolute_accZ'].insert(0, f"{local_absolute_acc[2]:.4f}")

            #按钮显示倒计时
            if recording_gravity:
                remaining_time = recording_duration - (time.time() - recording_start_time)
                calibrate_button['text'] = f"重力标定 ({remaining_time:.1f} 秒)"
                if remaining_time <= 0:
                    recording_gravity = False
                    calibrate_button['text'] = "重力标定"
                    calibrate_gravity()
                    #位移和速度清零
                    integral_velocity = np.array([0.0, 0.0, 0.0])
                    integral_position = np.array([0.0, 0.0, 0.0])
                    #初始化卡尔曼滤波器
                    kf = KalmanFilter(F, B, H, Q, R, x0, P0)
                    print("重力标定完成。")

            #显示本地速度和位移
            entries['local_velocityX'].delete(0, tk.END)
            entries['local_velocityX'].insert(0, f"{integral_velocity[0]:.4f}")
            entries['local_velocityY'].delete(0, tk.END)
            entries['local_velocityY'].insert(0, f"{integral_velocity[1]:.4f}")
            entries['local_velocityZ'].delete(0, tk.END)
            entries['local_velocityZ'].insert(0, f"{integral_velocity[2]:.4f}")

            entries['local_positionX'].delete(0, tk.END)
            entries['local_positionX'].insert(0, f"{integral_position[0]:.4f}")
            entries['local_positionY'].delete(0, tk.END)
            entries['local_positionY'].insert(0, f"{integral_position[1]:.4f}")
            entries['local_positionZ'].delete(0, tk.END)
            entries['local_positionZ'].insert(0, f"{integral_position[2]:.4f}")



    except Exception as e:
        print(f"更新GUI错误: {e}")

    # 每隔10ms更新一次GUI
    root.after(50, update_gui)

# 启动GUI更新
update_gui()
# 在后台启动HTTP服务器
# uvicorn.run(app, host="0.0.0.0", port=59666)
#在新线程运行
threading.Thread(target=uvicorn.run, args=(app,), kwargs={'host': "0.0.0.0", 'port': 59666}, daemon=True).start()

# 运行主循环
try:
    root.mainloop()
except KeyboardInterrupt:
    print("程序已停止。")
finally:
    ser.close()  # 关闭串口