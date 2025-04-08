#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <ModbusMaster.h>
#include <BluetoothSerial.h>
#include <math.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

// ------------------------ 全局变量与常量 ------------------------
// OLED 屏幕参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// RS485 通信波特率
#define RS485_BAUD 57600

#define SSID "ESP32_AP"
#define PASSWORD "12345678"

#define ONE_ROLL 1000
#define HISTORY_SIZE 64  // 历史数据缓冲区大小
#define RECONNECT_INTERVAL 5000

MPU6050 mpu;
ModbusMaster node;
BluetoothSerial SerialBT;  // 声明蓝牙串口实例

WiFiServer wifiServer(8080);
WiFiClient wifiClient;

// 定义其他全局变量（例如伺服电机参数等）
bool btConnected = false;  // 蓝牙连接状态标志
bool debugmode = false;
bool useTrapezoidalProfile = true;
bool start = true;
bool btPreviouslyConnected = false;

float originAngle;
float currentAngle;

// 当前速度和加速度
float currentSpeed;
float currentAcceleration;

float speedHistory[HISTORY_SIZE];
float maxSpeed = 0.0f;

int historyIndex = 0;
unsigned long reconnectAttemptTime = 0;

float sensitivity;
unsigned int offset = 0;

int mode = 1;
// ------------------------ 模块函数声明 ------------------------
void initHardware();
void processRS485Communication();
void processSensors(float &modifiedSpeed, float &modifiedAcceleration);
void processControl();
void regulateControl(float modifiedSpeed, float modifiedAcceleration);
void processDisplay();
void processWireless();
void sendServoCommand();
void readServoResponse();
void handleCommand(String cmd);
void stopServo();
void debug();
void initTorqueMode();
void processMPU6050();
void mode1();
void mode2();
void mode3();

// PID控制器类
class PID {
public:
  // 默认构造函数
  PID()
    : kp(0), ki(0), kd(0), prevError(0), integral(0) {}

  // 初始化函数，用于在setup中设置PID参数
  void init(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    prevError = 0;
    integral = 0;
  }

  // 计算PID控制输出
  float compute(float setpoint, float input) {
    float error = setpoint - input;          // 计算误差
    float pTerm = kp * error;                // 比例项
    integral += error;                       // 积分累加
    float iTerm = ki * integral;             // 积分项
    float dTerm = kd * (error - prevError);  // 微分项
    float output = pTerm + iTerm + dTerm;    // 计算PID输出
    prevError = error;                       // 更新前一误差
    return output;
  }

private:
  float kp;         // 比例常数
  float ki;         // 积分常数
  float kd;         // 微分常数
  float prevError;  // 上一次的误差
  float integral;   // 积分值
};

// 全局PID控制器实例
PID pidSpeedT;         // 用于速度控制的PID 梯形曲线
PID pidAccelerationT;  // 用于加减速控制的PID
PID pidSpeedS;         // 用于速度控制的PID s曲线
PID pidAccelerationS;  // 用于加减速控制的PID

class TrapezoidalProfile {
private:
  float t_acc;     // 加速阶段时间（秒）
  float t_const;   // 匀速阶段时间（秒）
  float t_dec;     // 减速阶段时间（秒）
  float maxSpeed;  // 最大速度（rpm）
  float a;         // 加速度（单位：0.1 rps/s）
  float d;         // 减速度（单位：0.1 rps/s）

public:
  // 构造函数：参数说明——
  // t_acc, t_const, t_dec：时间（秒）；maxSpeed 单位 rpm；a 和 d 单位为 0.1 rps/s
  TrapezoidalProfile(float t_acc, float t_const, float t_dec, float maxSpeed, float a, float d) {
    this->t_acc = t_acc;
    this->t_const = t_const;
    this->t_dec = t_dec;
    this->maxSpeed = maxSpeed;
    this->a = a;
    this->d = d;
  }

  // 根据当前时间 t（秒）计算目标速度（rpm）和目标加速度（0.1 rps/s）
  void updateProfile(float t, float &targetSpeed, float &targetAcceleration) {
    // 将 a 和 d 从 0.1 rps/s 单位转换为 rpm/s：乘以 6（因为 0.1 rps = 6 rpm）
    float accel_rpm_per_s_a = a * 6;
    float accel_rpm_per_s_d = d * 6;

    if (t < t_acc) {
      // 加速阶段：保持目标加速度为 a (0.1 rps/s)
      targetAcceleration = a;
      // 目标速度 = (加速度换算成 rpm/s) × t
      targetSpeed = accel_rpm_per_s_a * t;
    } else if (t < t_acc + t_const) {
      // 匀速阶段
      targetAcceleration = 0;
      targetSpeed = maxSpeed;
    } else if (t < t_acc + t_const + t_dec) {
      // 减速阶段：线性减速
      float t_dec_phase = t - t_acc - t_const;
      targetAcceleration = -d;
      // 目标速度 = 最大速度 - (减速度换算成 rpm/s × t_dec_phase)
      targetSpeed = maxSpeed - accel_rpm_per_s_d * t_dec_phase;
    } else {
      // 运动结束
      targetAcceleration = 0;
      targetSpeed = 0;
    }
  }
};

class SCurveProfile {
private:
  float t_r;       // 上升斜坡阶段时间（秒）
  float t_const;   // 恒定加速度阶段时间（秒）
  float t_r_down;  // 下降斜坡阶段时间（秒）
  float A_max;     // 最大加速度，单位为 0.1 rps/s
  float t_cv;      // 匀速阶段持续时间（秒）
  float T_acc;     // 加速阶段总时间 = t_r + t_const + t_r_down
  float V_max;     // 最大速度，单位 rpm

  // 辅助函数：计算加速阶段（0 ≤ t ≤ T_acc）的目标速度和加速度
  // 计算结果：speed 的单位为 0.1 rps，acceleration 单位为 0.1 rps/s
  void accelerationPhase(float t, float &speed, float &acceleration) {
    if (t < t_r) {
      // 上升斜坡阶段：加速度从 0 线性增加到 A_max
      acceleration = A_max * (t / t_r);
      speed = 0.5 * A_max * (t * t / t_r);
    } else if (t < t_r + t_const) {
      // 恒定加速度阶段：加速度恒定为 A_max
      acceleration = A_max;
      float speed_ramp = 0.5 * A_max * t_r;  // 上升阶段累计速度
      float t_const_phase = t - t_r;
      speed = speed_ramp + A_max * t_const_phase;
    } else if (t < T_acc) {
      // 下降斜坡阶段：加速度从 A_max 线性下降到 0
      float t_down = t - t_r - t_const;
      acceleration = A_max * (1 - t_down / t_r_down);
      float speed_ramp = 0.5 * A_max * t_r;
      float speed_const = A_max * t_const;
      float speed_ramp_down = A_max * t_down - 0.5 * A_max * (t_down * t_down / t_r_down);
      speed = speed_ramp + speed_const + speed_ramp_down;
    } else {
      acceleration = 0;
      // 若 t 超出加速阶段，直接返回最大速度（单位：0.1 rps）
      speed = V_max / 6.0;  // V_max 已经转换为 rpm，此处换回0.1 rps单位
    }
    // 最后将 speed 转换为 rpm：
    // 1 (0.1 rps) = 6 rpm，所以
    speed = speed * 6;
  }

public:
  // 构造函数
  // 参数 t_r, t_const, t_r_down 单位秒；A_max 单位为 0.1 rps/s；t_cv 单位秒
  SCurveProfile(float t_r, float t_const, float t_r_down, float A_max, float t_cv) {
    this->t_r = t_r;
    this->t_const = t_const;
    this->t_r_down = t_r_down;
    this->A_max = A_max;
    this->t_cv = t_cv;
    T_acc = t_r + t_const + t_r_down;
    // 计算最大速度，原公式得到单位为 0.1 rps，再转换为 rpm（乘以 6）
    float V_max_0_1 = 0.5 * A_max * t_r + A_max * t_const + 0.5 * A_max * t_r_down;
    V_max = V_max_0_1 * 6;
  }

  // 根据当前时间 t（秒）计算整个 S 曲线运动的目标速度和加速度
  // 返回的 targetSpeed 单位为 rpm，targetAcceleration 单位为 0.1 rps/s
  void updateProfile(float t, float &targetSpeed, float &targetAcceleration) {
    float T_total = 2 * T_acc + t_cv;
    if (t < 0) {
      targetSpeed = 0;
      targetAcceleration = 0;
    } else if (t < T_acc) {
      // 加速阶段
      float speed0, accel;
      accelerationPhase(t, speed0, accel);
      targetSpeed = speed0;        // 单位 rpm
      targetAcceleration = accel;  // 单位 0.1 rps/s
    } else if (t < T_acc + t_cv) {
      // 匀速阶段
      targetSpeed = V_max;
      targetAcceleration = 0;
    } else if (t < T_total) {
      // 减速阶段：对称于加速阶段
      float t_dec = t - (T_acc + t_cv);  // 当前减速阶段时间
      float t_mirror = T_acc - t_dec;    // 对称于加速阶段
      float speed_mirror, accel_mirror;
      accelerationPhase(t_mirror, speed_mirror, accel_mirror);
      targetSpeed = V_max - speed_mirror;
      targetAcceleration = -accel_mirror;
    } else {
      // 运动结束
      targetSpeed = 0;
      targetAcceleration = 0;
    }
  }
};
TrapezoidalProfile trapezoidal(0.5, 0.5, 0.5, 30, 10, 10);
SCurveProfile sCurve(0.5, 0, 0.5, 10, 0.5);

// ------------------------ setup() 函数 ------------------------
void setup() {
  initHardware();

  uint8_t result = node.readHoldingRegisters(0x2035, 2);
  if (result == node.ku8MBSuccess) {
    Serial.println("Read registers successfully:");
    Serial.print("Register 0: ");
    Serial.println(node.getResponseBuffer(0));
    Serial.print("Register 1: ");
    Serial.println(node.getResponseBuffer(1));
    // node.clearResponseBuffer();
  } else {
    Serial.print("Error reading registers, error code: ");
    Serial.println(result);
  }

  //初始化参数
  start = true;

  uint8_t ScaleGyroRange = mpu.getFullScaleGyroRange();
  switch (ScaleGyroRange) {
    case 0: sensitivity = 131.0; break;  // ±250°/s
    case 1: sensitivity = 65.5; break;   // ±500°/s
    case 2: sensitivity = 32.8; break;   // ±1000°/s
    case 3: sensitivity = 16.4; break;   // ±2000°/s
    default: sensitivity = 131.0; break;
  }
  Serial.print("灵敏度：");
  Serial.println(sensitivity);

  // t curve 和 s curve分开来搞
  // 是否测试电机
  if (debugmode) {
    debug();
  }
}

// ------------------------ loop() 函数 ------------------------
void loop() {
  switch (mode) {
    case 1: mode1(); break;//180度 pid 转
    case 2: mode2(); break;//平衡 mpu6050
    case 3: mode3(); break;//前馈 多端位置
    default: break;
  }
}

// ------------------------ 模块函数实现 ------------------------
// 硬件初始化模块
void initHardware() {
  // 初始化串口调试
  Serial.begin(115200);

  // 初始化 I2C 总线（用于 OLED 和 MPU6050）
  Wire.begin();
  // 初始化 OLED 显示屏
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //Serial.println("OLED failed");
    while (1);
  }
  display.clearDisplay();
  display.display();
  // 初始化 MPU6050
  mpu.initialize();
  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed");
  // } else {
  //   Serial.println("MPU6050 connection successful");
  // }

  // 初始化 RS485 通信：ESP32 的 Serial2 设置为 RX->D16, TX->D17
  Serial2.begin(RS485_BAUD, SERIAL_8N1, 17, 16);
  node.begin(01, Serial2);
  Serial.println("Modbus RTU initialization complete.");
  node.writeSingleRegister(0x2008, 1);  //复位操作
  delay(50);
  node.writeSingleRegister(0x2010, 1);  //复位操作
  delay(50);
  // node.writeSingleRegister(0x200A, 1);  //恢复出厂设置
  // delay(50);
  // node.writeSingleRegister(0x2101, 1);  //位置清零
  delay(1000);

  // 蓝牙初始化
  if (!SerialBT.begin("ESP32_Servo")) {  // 设备名称
    Serial.println("蓝牙初始化失败!");
  } else {
    Serial.println("蓝牙已就绪，名称: ESP32_Servo");
    SerialBT.setPin("1234", 4);  // 设置配对密码
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  }

  // Wifi初始化
  WiFi.softAP(SSID, PASSWORD);
  Serial.println("热点已启动");
  Serial.print("SSID: ");
  Serial.println(SSID);
  Serial.print("IP地址: ");
  Serial.println(WiFi.softAPIP());  // 默认IP通常是192.168.4.1
}  // end of initHardware


//-------MODE1 pid转180------------
void mode1() {
  float modifiedSpeed = 0;
  float modifiedAcceleration = 0;

  if (start) {
    processControl();
    // PID控制器实例
    pidSpeedT.init(0, 0.00, 0.00);         // 用于速度控制的PID 梯形曲线
    pidAccelerationT.init(0, 0.00, 0.00);  // 用于加减速控制的PID
    pidSpeedS.init(2, 0.05, 0.02);         // 用于速度控制的PID s曲线
    pidAccelerationS.init(2, 0.05, 0.01);  // 用于加减速控制的PID

    start = !start;
    offset = millis();  //记录初始时间，用于计算当前时间的理论速度和加速度
    Serial.print("初始时间：");
    Serial.println(offset / 1000);

    delay(20);
    uint8_t result = node.readHoldingRegisters(0x6077, 2);
    if (result == node.ku8MBSuccess) {
      uint16_t highWord = node.getResponseBuffer(0);  // 高16位
      uint16_t lowWord = node.getResponseBuffer(1);   // 低16位

      // 合并为一个32位数据（注意数据的符号问题）
      int32_t data = ((int32_t)highWord << 16) | lowWord;
      originAngle = data / 1000 * 360;  //记录初始角度，用于计算是否旋转了180°
      // node.clearResponseBuffer();
      Serial.print("初始角度：");
      Serial.println(originAngle);
    }

  }

  processSensors(modifiedSpeed, modifiedAcceleration);
  regulateControl(modifiedSpeed, modifiedAcceleration);

  processDisplay();
  processWireless();

  // delay(20);
  // uint8_t result = node.readHoldingRegisters(0x6077, 2);
  // if (result == node.ku8MBSuccess) {
  //   uint16_t highWord = node.getResponseBuffer(0);  // 高16位
  //   uint16_t lowWord = node.getResponseBuffer(1);   // 低16位

  //   // 合并为一个32位数据（注意数据的符号问题）
  //   int32_t data = ((int32_t)highWord << 16) | lowWord;
  //   currentAngle = data / ONE_ROLL * 360;  //记录角度，用于计算是否旋转了180°
  //   Serial.print("当前角度：");
  //   Serial.println(currentAngle);
  //   // node.clearResponseBuffer();
  // }


  // if (180 < currentAngle - originAngle) {
  //   start = true;
  //   useTrapezoidalProfile = !useTrapezoidalProfile;
  //   stopServo();

  //   delay(5000);
  //   clearSpeedHistory();
  //   //可以写一些后续的其他操作……
  // }
}
// 传感器模块：读取速度并计算PID调整值
void processSensors(float &modifiedSpeed, float &modifiedAcceleration) {
  float targetSpeed, targetAcceleration;

  delay(20);
  uint8_t result = node.readHoldingRegisters(0x606C, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t highWord = node.getResponseBuffer(0);  // 高16位
    uint16_t lowWord = node.getResponseBuffer(1);   // 低16位
    int32_t data = ((int32_t)highWord << 16) | lowWord;
    Serial.print("data:");
    Serial.println(data);
    currentSpeed = data;
    //node.clearResponseBuffer();
  }
  delay(20);
  result = node.readHoldingRegisters(0x606C, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t highWord = node.getResponseBuffer(0);  // 高16位
    uint16_t lowWord = node.getResponseBuffer(1);   // 低16位
    int32_t data = ((int32_t)highWord << 16) | lowWord;
    currentAcceleration = (data - currentSpeed) * 5000 / (60 * 50);
    //node.clearResponseBuffer();
  }
  float currentTime = (millis() - offset) / 1000.0;  // 当前时间（秒）


  if (useTrapezoidalProfile) {
    trapezoidal.updateProfile(currentTime, targetSpeed, targetAcceleration);
  } else {
    sCurve.updateProfile(currentTime, targetSpeed, targetAcceleration);
  }
  if (true) {
    // Serial.print("currentTime:");
    // Serial.println(currentTime);
    Serial.print("currentSpeed:");
    Serial.println(currentSpeed);
    Serial.print("currentAcceleration:");
    Serial.println(currentAcceleration);
    Serial.print("Target Speed: ");
    Serial.println(targetSpeed);
    Serial.print("Target Acceleration: ");
    Serial.println(targetAcceleration);
  }

  // 使用PID计算修正值
  if (useTrapezoidalProfile) {
    modifiedSpeed = pidSpeedT.compute(targetSpeed, currentSpeed);
    modifiedAcceleration = pidAccelerationT.compute(targetAcceleration, currentAcceleration);
  } else {
    modifiedSpeed = pidSpeedS.compute(targetSpeed, currentSpeed);
    modifiedAcceleration = pidAccelerationS.compute(targetAcceleration, currentAcceleration);
  }
}
void processControl() {
  node.writeSingleRegister(0x2109, 2);
  delay(50);
  node.writeSingleRegister(0x2380, 1);
  delay(50);
  node.writeSingleRegister(0x2382, 1);
  delay(50);
  node.writeSingleRegister(0x2385, 10);
  delay(50);
  node.writeSingleRegister(0x2390, 30);  //第一段
  delay(50);
  node.writeSingleRegister(0x2391, 10);
  delay(50);
  node.writeSingleRegister(0x2300, 2);
  delay(50);
}
// 控制模块：根据传入的修改后的速度和加速度设置Modbus寄存器
void regulateControl(float modifiedSpeed, float modifiedAcceleration) {
  modifiedAcceleration += currentAcceleration;
  //modifiedSpeed += currentSpeed;
  node.writeSingleRegister(0x2385, modifiedAcceleration);
  //delay(20);
  //node.writeSingleRegister(0x2390, modifiedSpeed);

  // 打印输出
  // Serial.print("Modified Speed: ");
  // Serial.println(modifiedSpeed);
  // Serial.print("Modified Acceleration: ");
  // Serial.println(modifiedAcceleration);
}
void stopServo() {
  node.writeSingleRegister(0x2300, 1);
  delay(50);
}


//----------MODE2 保持平衡---------------
void mode2() {
  if (start) {
    initTorqueMode();
    start = !start;
  }
  processMPU6050();
}
void initTorqueMode() {
  node.writeSingleRegister(0x2109, 2);
  delay(50);
  node.writeSingleRegister(0x23F1, 100);
  delay(50);
  node.writeSingleRegister(0x23F2, 50);
  delay(50);
  node.writeSingleRegister(0x23F3, 0);
  delay(50);
  node.writeSingleRegister(0x2300, 2);
  delay(50);
}
void processMPU6050() {
  int16_t ax, ay, az, gx, gy, gz;
  // 读取 MPU6050 的加速度、陀螺仪数据
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 根据加速度数据计算倾斜角（以 pitch 为例）
  // 采用 arctan2(accelX, accelZ) 计算倾角，单位转换为度
  float pitch = atan2((float)ax, (float)az) * 180.0 / PI;

  // 设定期望的水平角度为 0°
  float error = pitch;  // 如果当前角度偏离0，则 error 为正或负

  // 采用简单比例控制计算目标转矩（单位为百分比，注意转矩单位为 0.1%，例如 100 表示 10%）
  float Kp = 1.0;                    // 比例增益，实际值需调节
  float torquePercent = Kp * error;  // 假设1°偏差对应1%的转矩调整

  // 为了与寄存器单位匹配，将转矩百分比除以 0.1得到整数值
  int16_t torqueReg = (int16_t)(torquePercent / 0.1);

  // 将计算的目标转矩写入目标转矩寄存器（假设 0x23F3 为目标转矩，单位为 0.1%）
  uint8_t result = node.writeSingleRegister(0x23F3, (uint16_t)torqueReg);
  delay(50);

  // 调试输出
  Serial.print("MPU6050 Pitch: ");
  Serial.print(pitch);
  Serial.print(" deg, Error: ");
  Serial.print(error);
  Serial.print(" deg, Torque Command: ");
  Serial.print(torqueReg);
  Serial.println(" (0.1%)");
}


//-------MODE3 前馈模式------------
void mode3(){
  //feedfoward
    //多段位置模式实现，但是不能实时根据pid纠正速度、加速度
  node.writeSingleRegister(0x2109, 1);
  delay(50);
  node.writeSingleRegister(0x2310, 0);
  delay(50);
  node.writeSingleRegister(0x2311, 0);
  delay(50);
  node.writeSingleRegister(0x2314, 4);
  delay(50);
  node.writeSingleRegister(0x2315, 1);
  delay(50);

  int32_t displacement = 125;  // 第1段位移
  node.setTransmitBuffer(1, lowWord(displacement));
  node.setTransmitBuffer(0, highWord(displacement));
  node.writeMultipleRegisters(0x2320, 2);  // 写入0x2320及后续寄存器（共2个寄存器）
  delay(50);
  node.clearTransmitBuffer();
  node.writeSingleRegister(0x2321, 30);  //第1段目标速度
  delay(50);
  node.writeSingleRegister(0x2322, 10);  //第1段加速度
  delay(50);
  node.writeSingleRegister(0x2323, 0);  //第1段减速度
  delay(50);
  node.writeSingleRegister(0x2324, 0);  //第1段完成后等待时间
  delay(50);

  displacement = 250;  // 第2段位移
  node.setTransmitBuffer(1, lowWord(displacement));
  node.setTransmitBuffer(0, highWord(displacement));
  node.writeMultipleRegisters(0x2325, 2);
  delay(50);
  node.clearTransmitBuffer();
  node.writeSingleRegister(0x2326, 30);
  delay(50);
  node.writeSingleRegister(0x2327, 0);
  delay(50);
  node.writeSingleRegister(0x2328, 0);
  delay(50);
  node.writeSingleRegister(0x2329, 0);
  delay(50);

  displacement = 125;  // 第3段位移
  node.setTransmitBuffer(1, lowWord(displacement));
  node.setTransmitBuffer(0, highWord(displacement));
  node.writeMultipleRegisters(0x232A, 2);
  delay(50);
  node.clearTransmitBuffer();
  node.writeSingleRegister(0x232B, 1);
  delay(50);
  node.writeSingleRegister(0x232C, 0);
  delay(50);
  node.writeSingleRegister(0x232D, 10);
  delay(50);
  node.writeSingleRegister(0x232E, 0);
  delay(50);

  displacement = 0;  // 第4段位移
  node.setTransmitBuffer(1, lowWord(displacement));
  node.setTransmitBuffer(0, highWord(displacement));
  node.writeMultipleRegisters(0x232F, 2);
  delay(50);
  node.clearTransmitBuffer();
  node.writeSingleRegister(0x2330, 0);
  delay(50);
  node.writeSingleRegister(0x2331, 0);
  delay(50);
  node.writeSingleRegister(0x2332, 1);
  delay(50);
  node.writeSingleRegister(0x2333, 1000);
  delay(50);

  node.writeSingleRegister(0x2300, 2);
  delay(50);
}


// 显示模块：更新 OLED 显示屏内容
// 主显示函数
void processDisplay() {
  display.clearDisplay();

  // 1. 获取传感器数据
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float AccXangle = atan2((float)ay, sqrt(pow((float)ax, 2) + pow((float)az, 2))) * 180 / PI;
  updateSpeedHistory(currentSpeed);

  // 第2行：速度
  display.setCursor(0, 10);
  display.printf("Speed: %5.1f rpm", currentSpeed);

  // 3. 中间图表区 (速度曲线)
  const int graphY = 32;       // 图表起始Y坐标
  const int graphHeight = 28;  // 图表高度
  const int graphWidth = display.width();

  // 绘制坐标轴
  display.drawFastHLine(0, graphY + graphHeight / 2, graphWidth, SSD1306_WHITE);

  // 自动缩放因子 (基于最大速度)
  float scaleFactor = (graphHeight / 2 - 2) / maxSpeed;

  // 绘制速度曲线
  int prevX = 0;
  int prevY = graphY + graphHeight / 2 - (int)(speedHistory[0] * scaleFactor);

  for (int i = 1; i < HISTORY_SIZE; i++) {
    int x = map(i, 0, HISTORY_SIZE - 1, 0, graphWidth - 1);
    int y = graphY + graphHeight / 2 - (int)(speedHistory[i] * scaleFactor);

    // 只连接有效数据点
    if (i <= historyIndex || historyIndex == 0) {
      display.drawLine(prevX, prevY, x, y, SSD1306_WHITE);
    }
    prevX = x;
    prevY = y;
  }

  // 5. 更新显示
  display.display();

  // 定期重置最大速度（每10秒）
  static unsigned long lastReset = 0;
  if (millis() - lastReset > 10000) {
    maxSpeed = 0.0f;
    lastReset = millis();
  }
}

// 更新速度历史记录
void updateSpeedHistory(float speed) {
  speedHistory[historyIndex] = speed;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  // 更新最大速度（用于自动缩放）
  maxSpeed = max(maxSpeed, fabs(speed));
  if (maxSpeed < 5.0f) maxSpeed = 5.0f;  // 最小刻度
}

// 清空速度历史
void clearSpeedHistory() {
  for (int i = 0; i < HISTORY_SIZE; i++) {
    speedHistory[i] = 0.0f;
  }
  historyIndex = 0;
  maxSpeed = 0.0f;
}

// 无线通信模块
void processWireless() {
  // 检查连接状态变化
  bool currentConnected = SerialBT.hasClient();

  // 连接状态改变时的处理
  if (currentConnected != btConnected) {
    btConnected = currentConnected;
    Serial.println(btConnected ? "蓝牙已连接" : "蓝牙已断开");

    // 在OLED上显示蓝牙状态
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(btConnected ? "BT: Connected" : "BT: Disconnected");

    // 连接成功时发送欢迎消息
    if (btConnected) {
      SerialBT.println("CONNECTED:ESP32_Servo ready");
      btPreviouslyConnected = true;
    }
  }

  // 如果之前连接过但现在断开，尝试重新广播以便Windows重新连接
  if (btPreviouslyConnected && !btConnected) {
    unsigned long currentTime = millis();
    if (currentTime - reconnectAttemptTime > RECONNECT_INTERVAL) {
      Serial.println("尝试重新广播蓝牙...");
      // 重新设置蓝牙可见性
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      reconnectAttemptTime = currentTime;
    }
  }

  // 处理蓝牙数据
  while (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();
    //Serial.print("蓝牙消息：");
    //Serial.println(command);
    handleCommand(command);
  }

  // 如果已连接，周期性发送心跳以保持连接
  static unsigned long lastHeartbeatTime = 0;
  const unsigned long HEARTBEAT_INTERVAL = 2000;  // 2秒

  if (btConnected) {
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeatTime > HEARTBEAT_INTERVAL) {
      SerialBT.println("HEARTBEAT:" + String(currentTime / 1000));
      SerialBT.println(getServoDataStr());
      lastHeartbeatTime = currentTime;
    }
  }
}


/**
 * 消费蓝牙消息
 */
void handleCommand(String cmd) {
  if (cmd == "ROTATE") {
    // TODO：实现电机旋转180度
  } else if (cmd == "GET_DATA") {
    String dataStr = getServoDataStr();
    SerialBT.print(dataStr);
    //Serial.println("Sent sensor data: " + dataStr);
  } else {
    Serial.print("==========> 蓝牙命令不存在：");
    Serial.println(cmd);
  }

  // 处理WiFi连接
  if (!wifiClient || !wifiClient.connected()) {
    wifiClient = wifiServer.available();
  }

  // 处理WiFi数据
  if (wifiClient && wifiClient.connected()) {
    while (wifiClient.available()) {
      String command = wifiClient.readStringUntil('\n');
      command.trim();
      Serial.print("WiFi消息：");
      Serial.println(command);
      handleCommand(command);
    }
  }
}

// 发送传感器数据
void sendSensorData() {
  String dataStr = getServoDataStr();
  // 通过蓝牙发送
  if (btConnected) {
    SerialBT.print(dataStr);
  }

  // 通过WiFi发送
  if (wifiClient && wifiClient.connected()) {
    wifiClient.print(dataStr);
  }

  Serial.println("发送传感器数据: " + dataStr);
}

String getServoDataStr() {
  return "DATA:" + String(currentSpeed) + ":" + String(currentAcceleration);
}

//----------测试部分------------
void debug() {
  // 发送调试命令给伺服电机
  sendServoCommand();
  delay(50);  // 等待伺服电机响应
  readServoResponse();
  delay(1000);
  stopServo();
}

/**
 * 发送伺服电机调试命令
 */
void sendServoCommand() {

  node.writeSingleRegister(0x2109, 2);  // 设置模式
  delay(50);
  node.writeSingleRegister(0x2380, 2);
  delay(50);
  node.writeSingleRegister(0x2385, 100);
  delay(50);
  node.writeSingleRegister(0x2390, 100);  // 设定目标速度
  delay(50);
  uint8_t result = node.writeSingleRegister(0x2384, 0);  // 设定加速度为 10 rps/s
  delay(50);

  if (result == node.ku8MBSuccess) {
    Serial.println("Command sent successfully.");
  } else {
    Serial.print("Command failed with error code: ");
    Serial.println(result);
  }


/**
 * 读取伺服电机响应数据并输出到串口监视器
 */
void readServoResponse() {
  if (Serial2.available()) {
    Serial.println("receive servo response:");
    while (Serial2.available()) {
      char c = Serial2.read();
      Serial.print(c);
    }
  }
}


void processWifiServer() {
  if (!wifiClient || !wifiClient.connected()) {
    wifiClient = wifiServer.available();
    if (wifiClient) {
        Serial.println("New WiFi client connected");
    }
  }

  // 处理 WiFi 客户端数据
  if (wifiClient && wifiClient.connected()) {
    while (wifiClient.available()) {
        String command = wifiClient.readStringUntil('\n');
        command.trim();
        handleCommand(command);
    }
  }
}

// 发送传感器数据
void sendSensorData() {
  String dataStr = getServoDataStr();
  // 通过蓝牙发送
  if (btConnected) {
    SerialBT.print(dataStr);
  }

  // 通过WiFi发送
  if (wifiClient && wifiClient.connected()) {
    wifiClient.print(dataStr);
  }

  Serial.println("发送传感器数据: " + dataStr);
}

String getServoDataStr() {
  return "DATA:" + String(currentSpeed) + ":" + String(currentAcceleration);
}



//----------测试部分------------
void debug() {
  // 发送调试命令给伺服电机
  sendServoCommand();
  delay(50);  // 等待伺服电机响应
  readServoResponse();
  delay(1000);
  stopServo();
}

/**
 * 发送伺服电机调试命令
 */
void sendServoCommand() {

  node.writeSingleRegister(0x2109, 2);  // 设置模式
  delay(50);
  node.writeSingleRegister(0x2380, 2);
  delay(50);
  node.writeSingleRegister(0x2385, 100);
  delay(50);
  node.writeSingleRegister(0x2390, 100);  // 设定目标速度
  delay(50);
  uint8_t result = node.writeSingleRegister(0x2384, 0);  // 设定加速度为 10 rps/s
  delay(50);

  if (result == node.ku8MBSuccess) {
    Serial.println("Command sent successfully.");
  } else {
    Serial.print("Command failed with error code: ");
    Serial.println(result);
  }
}


/**
 * 读取伺服电机响应数据并输出到串口监视器
 */
void readServoResponse() {
  if (Serial2.available()) {
    Serial.println("receive servo response:");
    while (Serial2.available()) {
      char c = Serial2.read();
      Serial.print(c);
    }
    Serial.println();
  } else {
    Serial.println("no response");
  }
}
