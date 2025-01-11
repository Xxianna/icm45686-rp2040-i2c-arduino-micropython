import serial
import csv

# 配置串口
port = 'COM13'  # 串口号
baudrate = 9600  # 波特率
timeout = 1  # 超时时间

# CSV文件标题
csv_header = [
    "accX", "accY", "accZ",
    "gyroX", "gyroY", "gyroZ",
    "temperature",
    "postureW", "postureX", "postureY", "postureZ",
    "last_time"
]

# 打开串口
ser = serial.Serial(port, baudrate, timeout=timeout)

# 打开CSV文件准备写入
with open('recordtestA.csv', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    # 写入标题行
    csv_writer.writerow(csv_header)

    try:
        print(f"开始录制 {port} 的内容到 recordtestA.csv...")
        while True:
            # 读取串口数据
            data = ser.readline()
            if data:
                # 将数据解码并拆分为列表
                decoded_data = data.decode('utf-8').strip()
                data_list = decoded_data.split(',')  # 假设数据以逗号分隔

                # 检查数据长度是否匹配标题
                if len(data_list) == len(csv_header):
                    # 写入数据到CSV文件
                    csv_writer.writerow(data_list)
                    csvfile.flush()  # 确保数据立即写入文件
                    print(f"录制到: {data_list}")
                else:
                    print(f"数据格式不匹配，跳过: {data_list}")
                    exit()
    except KeyboardInterrupt:
        print("录制已停止。")
    finally:
        ser.close()  # 关闭串口