#include <Wire.h>
#include <Arduino.h>
#include <deque>
#include "pico/mutex.h"
#include "pico/stdlib.h"



volatile int zeroposition = 0;  // 使用 volatile 关键字，因为它在中断中被修改。用于参考姿态设置


#define I2C_ADDRESS 0x68
// ICM45686 类定义
class ICM45686 {
public:
    enum class Mode {
        Off = 0x00,
        Standby = 0x01,
        LowPower = 0x02,
        LowNoise = 0x03
    };

    enum class AccelScale {
        Scale32g = 0x00,
        Scale16g = 0x01,
        Scale8g = 0x02,
        Scale4g = 0x03,
        Scale2g = 0x04
    };

    enum class GyroScale {
        Scale4000dps = 0x00,
        Scale2000dps = 0x01,
        Scale1000dps = 0x02,
        Scale500dps = 0x03,
        Scale250dps = 0x04,
        Scale125dps = 0x05,
        Scale62dps = 0x06,
        Scale31dps = 0x07,
        Scale15dps = 0x08,
        Scale6dps = 0x09
    };

    enum class ODR {
        Rate6400Hz = 3,
        Rate3200Hz = 4,
        Rate1600Hz = 5,
        Rate800Hz = 6,
        Rate400Hz = 7,
        Rate200Hz = 8,
        Rate100Hz = 9,
        Rate50Hz = 10,
        Rate25Hz = 11,
        Rate12Hz = 12,
        Rate6Hz = 13,
        Rate3Hz = 14,
        Rate1Hz = 15
    };

    ICM45686(uint8_t i2c_address = I2C_ADDRESS, uint32_t freq = 100000) : i2c_address(i2c_address), accel_scale(32), gyro_scale(4000) {
        Wire1.setClock(freq);
    }

    bool begin() {
        Wire1.begin();
        return verify_connection();
    }

    bool verify_connection() {
        uint8_t who_am_i = read_register(0x72);
        return who_am_i == 0xE9;
    }

    bool accel_mode(Mode mode, AccelScale scale, ODR odr) {
        uint8_t pwr_mgmt0 = read_register(0x10);
        pwr_mgmt0 |= static_cast<uint8_t>(mode);
        write_register(0x10, pwr_mgmt0);

        uint8_t acc_config = read_register(0x1b);
        accel_scale = 1 << (5 - static_cast<uint8_t>(scale));
        acc_config = (acc_config & 0x80) | ((static_cast<uint8_t>(scale) & 0x07) << 4) | (static_cast<uint8_t>(odr) & 0x0F);
        write_register(0x1b, acc_config);
        return true;
    }

    bool gyro_mode(Mode mode, GyroScale scale, ODR odr) {
        uint8_t pwr_mgmt0 = read_register(0x10);
        pwr_mgmt0 |= static_cast<uint8_t>(mode) << 2;
        write_register(0x10, pwr_mgmt0);

        uint8_t gyro_config = read_register(0x1c);
        gyro_scale = 4000 / (1 << static_cast<uint8_t>(scale));
        gyro_config = ((static_cast<uint8_t>(scale) & 0x0F) << 4) | (static_cast<uint8_t>(odr) & 0x0F);
        write_register(0x1c, gyro_config);
        return true;
    }

    void get_data(float* accel_data, float* gyro_data, float* temp_data) {
        uint8_t raw_data[14];
        read_register(0x00, raw_data, 14);

        accel_data[0] = convert_accel((raw_data[1] << 8) | raw_data[0]);
        accel_data[1] = convert_accel((raw_data[3] << 8) | raw_data[2]);
        accel_data[2] = convert_accel((raw_data[5] << 8) | raw_data[4]);

        gyro_data[0] = convert_gyro((raw_data[7] << 8) | raw_data[6]);
        gyro_data[1] = convert_gyro((raw_data[9] << 8) | raw_data[8]);
        gyro_data[2] = convert_gyro((raw_data[11] << 8) | raw_data[10]);

        *temp_data = convert_temp((raw_data[13] << 8) | raw_data[12]);
    }

private:
    uint8_t i2c_address;
    float accel_scale;
    float gyro_scale;

    void write_register(uint8_t reg, uint8_t value) {
        Wire1.beginTransmission(i2c_address);
        Wire1.write(reg);
        Wire1.write(value);
        Wire1.endTransmission();
    }

    uint8_t read_register(uint8_t reg) {
        Wire1.beginTransmission(i2c_address);
        Wire1.write(reg);
        Wire1.endTransmission(false);
        Wire1.requestFrom(i2c_address, 1);
        return Wire1.read();
    }

    void read_register(uint8_t reg, uint8_t* buffer, uint8_t length) {
        Wire1.beginTransmission(i2c_address);
        Wire1.write(reg);
        Wire1.endTransmission(false);
        Wire1.requestFrom(i2c_address, length);
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = Wire1.read();
        }
    }

    float convert_accel(int16_t raw) {
        return ((float)raw * accel_scale) / 32768.0 * 9.80665;
    }

    float convert_gyro(int16_t raw) {
        return ((float)raw * gyro_scale * M_PI) / (32768.0 * 180.0);
    }

    float convert_temp(int16_t raw) {
        return raw / 128.0 + 25;
    }
};


// 四元数微分方程
void quaternion_derivative(float* q, float gx, float gy, float gz, float* dq) {
    dq[0] = 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz);
    dq[1] = 0.5 * (q[0] * gx + q[2] * gz - q[3] * gy);
    dq[2] = 0.5 * (q[0] * gy - q[1] * gz + q[3] * gx);
    dq[3] = 0.5 * (q[0] * gz + q[1] * gy - q[2] * gx);
}

// 使用 RK4 更新四元数
void update_quaternion_rk4(float* q, float* gyro_data, float dt) {
    float k1[4], k2[4], k3[4], k4[4];
    float q_temp[4];

    quaternion_derivative(q, gyro_data[0], gyro_data[1], gyro_data[2], k1);
    for (int i = 0; i < 4; i++) q_temp[i] = q[i] + 0.5 * dt * k1[i];
    quaternion_derivative(q_temp, gyro_data[0], gyro_data[1], gyro_data[2], k2);
    for (int i = 0; i < 4; i++) q_temp[i] = q[i] + 0.5 * dt * k2[i];
    quaternion_derivative(q_temp, gyro_data[0], gyro_data[1], gyro_data[2], k3);
    for (int i = 0; i < 4; i++) q_temp[i] = q[i] + dt * k3[i];
    quaternion_derivative(q_temp, gyro_data[0], gyro_data[1], gyro_data[2], k4);

    for (int i = 0; i < 4; i++) {
        q[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }

    float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) {
        q[i] /= norm;
    }
}




ICM45686 imu;
uint64_t last_time=0;
float posture[4] = {1.0, 0.0, 0.0, 0.0};

#define INTERVALTIME 5000 //200hz
const float mydt=(float)INTERVALTIME / (1000*1000);


void setup() {
  set_sys_clock_khz(250000, true);
  // 初始化串口通信，设置波特率为9600
  Serial.begin();

  Wire1.setSDA(2);
  Wire1.setSCL(3);

  if (!imu.begin()) {
      while (1){
        Serial.println("Failed to initialize ICM45686!");
        delay(300);
      }
  }
  imu.accel_mode(ICM45686::Mode::LowNoise, ICM45686::AccelScale::Scale16g, ICM45686::ODR::Rate200Hz); //降低白噪声
  imu.gyro_mode(ICM45686::Mode::LowNoise, ICM45686::GyroScale::Scale2000dps, ICM45686::ODR::Rate200Hz);

  
}


void loop() {
    if(time_us_64()<last_time){
      sleep_until(last_time);
    }else{
      Serial.println("Out of time------------------------------------------------");
      last_time = time_us_64();
    }

    float accel_data[3], gyro_data[3], temp_data;
    imu.get_data(accel_data, gyro_data, &temp_data);
    float correct_gyro[3];
    correct_gyro[0] = gyro_data[0];
    correct_gyro[1] = gyro_data[1];
    correct_gyro[2] = gyro_data[2];
    // correct_gyro[0] = gyro_data[0] - 0.001413;
    // correct_gyro[1] = gyro_data[1] + 0.000219;
    // correct_gyro[2] = gyro_data[2] - 0.000515;
    const float die_distance_gyro = 0.0055;
    if (correct_gyro[0] > -die_distance_gyro && correct_gyro[0] < die_distance_gyro)
        correct_gyro[0] = 0;
    if (correct_gyro[1] > -die_distance_gyro and correct_gyro[1] < die_distance_gyro)
        correct_gyro[1] = 0;
    if (correct_gyro[2] > -die_distance_gyro and correct_gyro[2] < die_distance_gyro)
        correct_gyro[2] = 0;
    if (zeroposition){
      posture[0]=1.0;
      posture[1]=0.0;
      posture[2]=0.0;
      posture[3]=0.0;
    }
    else
      update_quaternion_rk4(posture, correct_gyro, mydt);
    last_time+=INTERVALTIME;

    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d",
              accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2], temp_data,posture[0],posture[1],posture[2],posture[3],last_time);
    Serial.println(buffer);

}

String inputString = "";        // 用于存储串口接收到的字符串
bool stringComplete = false;    // 用于标识是否接收到完整的字符串

void setup1() {
  inputString.reserve(200);     // 为输入字符串预留内存

}

void loop1() {
  if (stringComplete) {
    // 按 ";" 分割字符串
    int delimiterIndex = inputString.indexOf(';');
    if (delimiterIndex != -1) {
      String command = inputString.substring(0, delimiterIndex);
      inputString = inputString.substring(delimiterIndex + 1);

      if (command == "zeroposition") {
        zeroposition = 1;       // 设置 zeroposition 为 1
        add_alarm_in_ms(100, [](alarm_id_t id, void *user_data) -> int64_t {
            *(int*)user_data = 0; // 将 zeroposition 置为 0
            return 0; // 返回 0 表示不重复定时器
        }, const_cast<void*>(reinterpret_cast<volatile void*>(&zeroposition)), true);
      }
    }
    inputString = "";           // 清空输入字符串
    stringComplete = false;     // 重置标志位
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();  // 读取串口数据
    inputString += inChar;              // 将字符添加到输入字符串
    if (inChar == '\n') {
      stringComplete = true;            // 如果接收到换行符，设置标志位
    }
  }
}
