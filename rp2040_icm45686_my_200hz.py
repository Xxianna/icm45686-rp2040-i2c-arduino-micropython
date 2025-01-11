from machine import I2C, Pin, Timer, UART
import time
import _thread
import machine
from collections import deque
import uasyncio as asyncio
import uos

machine.freq(250000000)

#I2C_MAX_CLOCK 1000000 # icm45686

class ICM45686:
    class Mode:
        Off = 0x00
        Standby = 0x01	#acc no this
        LowPower = 0x02
        LowNoise = 0x03

    class AccelScale:
        Scale32g = 0x00
        Scale16g = 0x01
        Scale8g = 0x02
        Scale4g = 0x03
        Scale2g = 0x04

    class GyroScale:
        Scale4000dps = 0x00
        Scale2000dps = 0x01
        Scale1000dps = 0x02
        Scale500dps = 0x03
        Scale250dps = 0x04
        Scale125dps = 0x05
        Scale62dps = 0x06
        Scale31dps = 0x07
        Scale15dps = 0x08
        Scale6dps = 0x09

    class ODR:
        Rate6400Hz = 3
        Rate3200Hz = 4
        Rate1600Hz = 5
        Rate800Hz = 6
        Rate400Hz = 7
        Rate200Hz = 8
        Rate100Hz = 9
        Rate50Hz = 10
        Rate25Hz = 11
        Rate12Hz = 12
        Rate6Hz = 13
        Rate3Hz = 14
        Rate1Hz = 15

    def __init__(self, i2c_address=0x68, freq=100000):
        self.i2c_address = i2c_address
        self.i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=freq)
        self.raw_data = bytearray(14)
        self.accel_scale = 32  # 默认加速度计量程为 32g
        self.gyro_scale = 2000  # 默认陀螺仪量程为 2000dps

    def begin(self):
        return self.verify_connection()

    def verify_connection(self):
        who_am_i = self.read_register(0x72, 1)
        return who_am_i[0] == 0xE9

    def accel_mode(self, mode, scale, odr):
        # 读取 PWR_MGMT0 寄存器
        pwr_mgmt0 = bytearray(self.read_register(0x10, 1))
        pwr_mgmt0[0] |= mode  # 设置加速度计模式
        self.write_register(0x10, pwr_mgmt0)
        # 更新加速度计量程和频率
        acc_config = bytearray(self.read_register(0x1b, 1))
        self.accel_scale = 2 ** (5 - scale)
        # 先清除 accel_ui_fs_sel 的旧值，然后设置新值
        acc_config[0] = (acc_config[0] & 0x80) | ((scale & 0x07) << 4) | (odr & 0x0F)  # 0x80 是掩码，保留最高位
        self.write_register(0x1b, acc_config)
        return True

    def gyro_mode(self, mode, scale, odr):
        # 读取 PWR_MGMT0 寄存器
        pwr_mgmt0 = bytearray(self.read_register(0x10, 1))
        pwr_mgmt0[0] |= mode << 2  # 设置陀螺仪模式
        self.write_register(0x10, pwr_mgmt0)

        # 设置陀螺仪配置
        gyro_config = bytearray(self.read_register(0x1c, 1))
        # 更新陀螺仪量程
        self.gyro_scale = 4000 / (2 ** scale)       
        gyro_config[0] = ((scale & 0x0F) << 4) | (odr & 0x0F)  # scale 在高 4 位，odr 在低 4 位
        self.write_register(0x1c, gyro_config)
        return True

    def get_data(self):
        # 读取 14 字节的原始数据（加速度计、陀螺仪和温度）
        self.raw_data = self.read_register(0x00, 14)

        # 解析加速度计数据
        accel_data = [
            self._to_signed_int((self.raw_data[1] << 8) | self.raw_data[0]),
            self._to_signed_int((self.raw_data[3] << 8) | self.raw_data[2]),
            self._to_signed_int((self.raw_data[5] << 8) | self.raw_data[4])
        ]

        # 解析陀螺仪数据
        gyro_data = [
            self._to_signed_int((self.raw_data[7] << 8) | self.raw_data[6]),
            self._to_signed_int((self.raw_data[9] << 8) | self.raw_data[8]),
            self._to_signed_int((self.raw_data[11] << 8) | self.raw_data[10])
        ]

        # 解析温度数据
        temp_data = [self._to_signed_int((self.raw_data[13] << 8) | self.raw_data[12])]

        return accel_data, gyro_data, temp_data

    def get_converted_data(self):
        accel_data, gyro_data, temp_data = self.get_data()

        # 转换加速度计数据
        converted_accel = [
            self._convert_accel(accel_data[0]),
            self._convert_accel(accel_data[1]),
            self._convert_accel(accel_data[2])
        ]

        # 转换陀螺仪数据
        converted_gyro = [
            self._convert_gyro(gyro_data[0]),
            self._convert_gyro(gyro_data[1]),
            self._convert_gyro(gyro_data[2])
        ]

        # 转换温度数据
        converted_temp = self._convert_temp(temp_data[0])

        return converted_accel, converted_gyro, converted_temp

    def _convert_accel(self, raw):
        """将加速度计原始数据转换为 m/s²"""
        return (raw * self.accel_scale) / 32768.0 * 9.80665

    def _convert_gyro(self, raw):
        """将陀螺仪原始数据转换为 rad/s"""
        return (raw * self.gyro_scale * 3.141592653589793) / (32768.0 * 180.0)

    def _convert_temp(self, raw):
        """将温度原始数据转换为 °C"""
        return raw / 128.0 + 25

    def write_register(self, reg, value):
        if isinstance(value, int):
            value = bytearray([value])
        self.i2c.writeto_mem(self.i2c_address, reg, value)

    def read_register(self, reg, length):
        return self.i2c.readfrom_mem(self.i2c_address, reg, length)

    def _to_signed_int(self, value):
        """将16位无符号整数转换为有符号整数"""
        if value & 0x8000:
            return value - 0x10000
        else:
            return value
        
        
        
# 定义四元数微分方程
def quaternion_derivative(q, gx, gy, gz):
    dq0 = 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz)
    dq1 = 0.5 * ( q[0] * gx + q[2] * gz - q[3] * gy)
    dq2 = 0.5 * ( q[0] * gy - q[1] * gz + q[3] * gx)
    dq3 = 0.5 * ( q[0] * gz + q[1] * gy - q[2] * gx)
    return [dq0, dq1, dq2, dq3]

# 使用 RK4 更新四元数
def update_quaternion_rk4(q, gyro_data, dt):
    gx, gy, gz = gyro_data[0], gyro_data[1], gyro_data[2]
    # 计算 k1
    k1 = quaternion_derivative(q, gx, gy, gz)
    
    # 计算 k2
    q_temp = [q[i] + 0.5 * dt * k1[i] for i in range(4)]
    k2 = quaternion_derivative(q_temp, gx, gy, gz)
    
    # 计算 k3
    q_temp = [q[i] + 0.5 * dt * k2[i] for i in range(4)]
    k3 = quaternion_derivative(q_temp, gx, gy, gz)
    
    # 计算 k4
    q_temp = [q[i] + dt * k3[i] for i in range(4)]
    k4 = quaternion_derivative(q_temp, gx, gy, gz)
    
    # 更新四元数
    for i in range(4):
        q[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
    
    # 归一化四元数
    norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2) ** 0.5
    for i in range(4):
        q[i] /= norm
        
    return q
        
        
        
        
        
        
        
        
        
        
# 创建两个队列和锁
queue1 = deque([],40)  # 用于快速写入的队列
queue2 = deque([],40)  # 用于异步打印的队列
queue1_lock = _thread.allocate_lock()  # 仅对 queue1 加锁       
        
# 合并后的任务函数
async def transfer_and_print_task():
    #uos.task_pin(0, True)
    while True:
        # 将 queue1 中的所有元素移动到 queue2
        if queue1_lock.acquire():  # 加锁 queue1
            if queue1:
                # 一次性将 queue1 的所有元素移动到 queue2
                while queue1:
                    data = queue1.popleft()
                    queue2.append(data)
                queue1_lock.release()  # 解锁 queue1
            else:
                queue1_lock.release()  # 解锁 queue1

        # 依次发送 queue2 中的所有元素
        while queue2:
            data_str = queue2.popleft()
            if data_str[1]==0:
                if data_str[0]<0:
                    print(data_str[0])
                else:
                    print(f"----------data_str{[0]}")
            else:
                formatted_str = "-----------------{:.0f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}".format(*data_str)
                print(formatted_str)

        await asyncio.sleep(0.05)  # 短暂休眠，避免占用过多 CPU

        

# 初始化 ICM45686
imu = ICM45686(freq = 100000)

# 设置 AD0 引脚
ad0_pin = Pin(3, Pin.OUT)
ad0_pin.value(0)  # I2C 地址为 0xE9

# 检查设备是否连接成功
if not imu.begin():
    print("Failed to initialize ICM45686!")
    exit()

# 设置加速度计和陀螺仪模式（可选）
imu.accel_mode(ICM45686.Mode.LowNoise, ICM45686.AccelScale.Scale8g, ICM45686.ODR.Rate200Hz)
imu.gyro_mode(ICM45686.Mode.LowNoise, ICM45686.GyroScale.Scale2000dps, ICM45686.ODR.Rate200Hz)

dt = 0.005
posture = [1.0, 0.0, 0.0, 0.0]  # 初始化四元数（其实需要手动初始化）
timestamp = 0  # 时间戳
tim = Timer() 

# 定义 IMU 读取任务
def imu_read_task(timer):
    global posture,timestamp
    #uos.task_pin(, True)
    start_time = time.ticks_us()
    # 读取 IMU 数据
    accel_data, gyro_data, temp_data = imu.get_converted_data()

    # 将数据存入环形队列
    #imu_data_queue.append((accel_data, gyro_data, temp_data))
    
    #更新计算四元数 TODO：需要加锁 TODO：消除误差
    posture = update_quaternion_rk4(posture, gyro_data, dt)
    
    # 格式化数据
    data_str = [timestamp,accel_data[0], accel_data[1], accel_data[2],gyro_data[0], gyro_data[1], gyro_data[2],posture[0], posture[1], posture[2], posture[3],temp_data]
        # 将数据快速写入 queue1（加锁保护）
    if queue1_lock.acquire():  # 加锁
        queue1.append(data_str)
        queue1_lock.release()  # 解锁
    #print(data_str)
    
    # 时间戳自增，使用2字节整数等待自然翻转
    timestamp = (timestamp + 1) & 0xFFFF
    
    # 计算剩余时间，确保严格的 200Hz 读取频率
    elapsed_time = time.ticks_diff(time.ticks_us(), start_time)
    sleep_time = 5000 - elapsed_time  # 5000us = 5ms (200Hz)
    #print(sleep_time)
    data_str = [sleep_time,0, 0, 0,0, 0, 0,0, 0, 0, 0,0]
        # 将数据快速写入 queue1（加锁保护）
    if queue1_lock.acquire():  # 加锁
        queue1.append(data_str)
        queue1_lock.release()  # 解锁
        
tim.init(period=5, mode=Timer.PERIODIC, callback=imu_read_task)

# 主循环中使用 asyncio 运行任务
async def main():
    # 启动打印任务
    asyncio.create_task(transfer_and_print_task())
    
    # 主循环
    while True:
        await asyncio.sleep(0.05)  # 防止主循环占用过多CPU

# 启动 asyncio 事件循环
asyncio.run(main())