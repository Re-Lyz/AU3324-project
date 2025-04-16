#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
//#include <MPU6050.h>
#include <MPU6050_tockn.h>
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
#define RS485_BAUD 115200

#define SSID "ESP32_AP"
#define PASSWORD "12345678"

#define ONE_ROLL 1000
#define HISTORY_SIZE 64  // 历史数据缓冲区大小
#define RECONNECT_INTERVAL 5000

MPU6050 mpu(Wire);
ModbusMaster node;
BluetoothSerial SerialBT;  // 声明蓝牙串口实例

WiFiServer wifiServer(8080);
WiFiClient wifiClient;

// 定义其他全局变量（例如伺服电机参数等）
bool btConnected = false;  // 蓝牙连接状态标志
bool useTrapezoidalProfile = true;
bool start1 = true;
bool start2 = true;
bool btPreviouslyConnected = false;

float originAngle;
float currentAngle;

// 当前参数
float OriginPos;
float currentPos;
float currentSpeed;
float currentAcceleration;

float speedHistory[HISTORY_SIZE];
float maxSpeed = 0.0f;

int historyIndex = 0;
unsigned long reconnectAttemptTime = 0;

float sensitivity;
unsigned int offset = 0;

int mode = 1;  //伺服电机运行模式
// ------------------------ 模块函数声明 ------------------------
void initHardware();
void processRS485Communication();
void processSensors(float &modifiedSpeed, float &modifiedPos);
void processControl();
void regulateControl(float modifiedSpeed, float modifiedPos);
float getPosition();
float getSpeed();
float getAcc();
void processDisplay();
void processWireless();
void sendServoCommand();
void readServoResponse();
void handleCommand(String cmd);
void stopServo();
void debug();
void initTorqueMode();
void processMPU6050(float &origin);
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
PID pidSpeedT;  // 用于速度控制的PID 梯形曲线
PID pidPosT;    // 用于加减速控制的PID
PID pidSpeedS;  // 用于速度控制的PID s曲线
PID pidPosS;    // 用于加减速控制的PID

PID pidPosmode2;
PID pidSpeedmode2;

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

  // 同时计算目标速度（rpm）、目标加速度（0.1 rps/s）和目标位置（单位：pul）
  void updateProfile(float t, float &targetSpeed, float &targetAcceleration, float &targetPosition) {
    // 转换因子：将速度（rpm）转换为脉冲/秒，假设 1 圈 = 1000 pul，1 分钟 = 60 秒
    float c = ONE_ROLL / 60.0;
    // 将 a 和 d 从 0.1 rps/s 单位转换为 rpm/s（乘以 6，因为 0.1 rps = 6 rpm）
    float accel_rpm_s = a * 6;
    float decel_rpm_s = d * 6;

    if (t < t_acc) {
      // 加速阶段
      targetAcceleration = a;
      targetSpeed = accel_rpm_s * t;
      // 位移为三角形面积： 1/2 * 加速度（rpm/s） * t^2，再转换为脉冲：乘以 c
      targetPosition = 0.5 * accel_rpm_s * t * t * c;
    } else if (t < t_acc + t_const) {
      // 匀速阶段
      targetAcceleration = 0;
      targetSpeed = maxSpeed;
      // 先计算加速阶段累计位移
      float pos_acc = 0.5 * accel_rpm_s * t_acc * t_acc * c;
      // 匀速阶段位移：速度(maxSpeed)转换为 pul/s：maxSpeed * c，再乘以匀速时间
      targetPosition = pos_acc + maxSpeed * c * (t - t_acc);
    } else if (t < t_acc + t_const + t_dec) {
      // 减速阶段
      float t_dec_phase = t - t_acc - t_const;
      targetAcceleration = -d;
      targetSpeed = maxSpeed - decel_rpm_s * t_dec_phase;
      float pos_acc = 0.5 * accel_rpm_s * t_acc * t_acc * c;
      float pos_const = maxSpeed * c * t_const;
      // 减速阶段位移：梯形面积，计算当前阶段的平均速度
      float pos_dec = ((maxSpeed + (maxSpeed - decel_rpm_s * t_dec_phase)) / 2.0) * c * t_dec_phase;
      targetPosition = pos_acc + pos_const + pos_dec;
    } else {
      // 运动结束：全部运动的累计位移
      targetAcceleration = 0;
      targetSpeed = 0;
      float pos_acc = 0.5 * accel_rpm_s * t_acc * t_acc * c;
      float pos_const = maxSpeed * c * t_const;
      float pos_dec = ((maxSpeed + (maxSpeed - decel_rpm_s * t_dec)) / 2.0) * c * t_dec;
      targetPosition = pos_acc + pos_const + pos_dec;
    }
  }
};

class SCurveProfile {
private:
  float t_r;                        // 上升斜坡阶段时间（秒）
  float t_const;                    // 恒定加速度阶段时间（秒）
  float t_r_down;                   // 下降斜坡阶段时间（秒）
  float A_max;                      // 最大加速度，单位为 0.1 rps/s
  float t_cv;                       // 匀速阶段持续时间（秒）
  float T_acc;                      // 加速阶段总时间 = t_r + t_const + t_r_down
  float V_max;                      // 最大速度，单位 rpm
  const float c = ONE_ROLL / 60.0;  // 转换因子，将 rpm 转换为脉冲/秒

  // 辅助函数：计算加速阶段时刻 t 内的目标速度和加速度
  // 输出 speed 单位：rpm；acceleration 单位：0.1 rps/s
  void accelerationPhase(float t, float &speed, float &acceleration) {
    if (t < t_r) {
      // 上升斜坡阶段：加速度从 0 线性增加到 A_max
      acceleration = A_max * (t / t_r);
      speed = 0.5 * A_max * (t * t / t_r);
    } else if (t < t_r + t_const) {
      // 恒定加速度阶段
      acceleration = A_max;
      float speed_ramp = 0.5 * A_max * t_r;
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
      // t 超出加速阶段返回最大速度（单位：0.1 rps，后续转换为 rpm）
      speed = V_max / 6.0;  // 注意：1 (0.1 rps)= 6 rpm
    }
    // 将 speed 转换为 rpm
    speed = speed * 6;
  }

  // 辅助函数：利用数值积分计算加速阶段累计位移（pul）
  float integratedAccelerationPhase(float t) {
    float dt = 0.001;  // 积分步长，单位秒
    float pos = 0;
    for (float tau = 0; tau < t; tau += dt) {
      float speed, accel;
      accelerationPhase(tau, speed, accel);
      // 将 speed（rpm）转换为脉冲/秒：speed * c，然后积分 dt 得到位移
      pos += speed * c * dt;
    }
    return pos;
  }

public:
  // 构造函数：参数单位：t_r, t_const, t_r_down, t_cv 为秒；A_max 为 0.1 rps/s
  SCurveProfile(float t_r, float t_const, float t_r_down, float A_max, float t_cv) {
    this->t_r = t_r;
    this->t_const = t_const;
    this->t_r_down = t_r_down;
    this->A_max = A_max;
    this->t_cv = t_cv;
    T_acc = t_r + t_const + t_r_down;
    // 计算最大速度（单位为 0.1 rps，转换为 rpm）
    float V_max_0_1 = 0.5 * A_max * t_r + A_max * t_const + 0.5 * A_max * t_r_down;
    V_max = V_max_0_1 * 6;
  }

  // 修改后的 updateProfile：输出目标速度 (rpm)、目标加速度 (0.1 rps/s)
  // 和目标位移（pul）
  void updateProfile(float t, float &targetSpeed, float &targetAcceleration, float &targetPosition) {
    float T_total = 2 * T_acc + t_cv;
    if (t < 0) {
      targetSpeed = 0;
      targetAcceleration = 0;
      targetPosition = 0;
    } else if (t < T_acc) {
      // 加速阶段
      float speed0, accel;
      accelerationPhase(t, speed0, accel);
      targetSpeed = speed0;
      targetAcceleration = accel;
      targetPosition = integratedAccelerationPhase(t);
    } else if (t < T_acc + t_cv) {
      // 匀速阶段
      targetSpeed = V_max;
      targetAcceleration = 0;
      float pos_acc = integratedAccelerationPhase(T_acc);
      targetPosition = pos_acc + V_max * c * (t - T_acc);
    } else if (t < T_total) {
      // 减速阶段：利用加速阶段的对称性
      float t_dec = t - (T_acc + t_cv);
      float mirrorTime = T_acc - t_dec;
      float speed_mirror, accel_mirror;
      accelerationPhase(mirrorTime, speed_mirror, accel_mirror);
      targetSpeed = V_max - speed_mirror;
      targetAcceleration = -accel_mirror;
      float pos_acc = integratedAccelerationPhase(T_acc);
      float pos_dec = pos_acc - integratedAccelerationPhase(mirrorTime);
      targetPosition = pos_acc + V_max * c * t_cv + pos_dec;
    } else {
      targetSpeed = 0;
      targetAcceleration = 0;
      float pos_acc = integratedAccelerationPhase(T_acc);
      targetPosition = 2 * pos_acc + V_max * c * t_cv;
    }
  }
};
TrapezoidalProfile trapezoidal(0.5, 0.5, 0.5, 30, 10, 10);
SCurveProfile sCurve(0.5, 0, 0.5, 10, 0);

// ------------------------ setup() 函数 ------------------------
void setup() {
  initHardware();

  const char *msg = "Hello, World!";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - w) / 2;
  int16_t y = (SCREEN_HEIGHT - h) / 2;

  display.setCursor(x, y);
  display.print(msg);
  display.display();


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
  start1 = true;
  start2 = true;

  // uint8_t ScaleGyroRange = mpu.getFullScaleGyroRange();
  // switch (ScaleGyroRange) {
  //   case 0: sensitivity = 131.0; break;  // ±250°/s
  //   case 1: sensitivity = 65.5; break;   // ±500°/s
  //   case 2: sensitivity = 32.8; break;   // ±1000°/s
  //   case 3: sensitivity = 16.4; break;   // ±2000°/s
  //   default: sensitivity = 131.0; break;
  // }
  // Serial.print("灵敏度：");
  // Serial.println(sensitivity);
}

// ------------------------ loop() 函数 ------------------------
void loop() {
  currentSpeed = getSpeed();
  processDisplay();
  processWireless();
  switch (mode) {
    case 1: mode1(); break;  //180度 pid 转
    case 2: mode2(); break;  //平衡 mpu6050
    case 3: mode3(); break;  //多端位置
    case 4: debug(); break;
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
    while (1)
      ;
  }
  display.clearDisplay();
  display.display();
  // 初始化 MPU6050
  mpu.begin();
  mpu.calcGyroOffsets(true);
  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed");
  // } else {
  //   Serial.println("MPU6050 connection successful");
  // }

  // 初始化 RS485 通信：ESP32 的 Serial2 设置为 RX->D16, TX->D17
  Serial2.begin(RS485_BAUD, SERIAL_8N1, 17, 16);
  node.begin(01, Serial2);
  Serial.println();
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

void mode1() {
  float modifiedSpeed = 0;
  float modifiedPos = 0;

  if (start1) {
    processControl();
    // PID控制器实例
    pidSpeedT.init(1, 0.00, 0.00);  // 用于速度控制的PID 梯形曲线
    pidPosT.init(1.0, 0.01, 0.01);  // 用于位置控制的PID
    pidSpeedS.init(1, 0.00, 0.00);  // 用于速度控制的PID s曲线
    pidPosS.init(1.0, 0.00, 0.5);   // 用于位置控制的PID
    delay(20);
    OriginPos = getPosition();
    start1 = !start1;
    offset = millis();  //记录初始时间，用于计算当前时间的理论速度和加速度
    Serial.print("开始时间：");
    Serial.println(offset / 1000);
    Serial.print("初始位置:");
    Serial.println(OriginPos);
    if (useTrapezoidalProfile) {
      Serial.println("T曲线模式");
    } else {
      Serial.println("S曲线模式");
    }
    delay(10);
    node.writeSingleRegister(0x2300, 2);
    delay(5);
  }

  processSensors(modifiedSpeed, modifiedPos);
  regulateControl(modifiedSpeed, modifiedPos);


  if (currentPos >= 500) {
    start1 = !start1;
    useTrapezoidalProfile = !useTrapezoidalProfile;
    stopServo();
    delay(5000);
    clearSpeedHistory();
    //可以写一些后续的其他操作……
  }
}

float origin = 0;
void mode2() {
  if (start2) {
    initTorqueMode();
    start2 = !start2;
    pidPosmode2.init(1, 0.01, 0.05);
    pidSpeedmode2.init(1, 0.0, 0.05);
    int16_t ax, ay, az, gx, gy, gz;
    mpu.update();
    origin = mpu.getAngleX();
    OriginPos = getPosition();
  }
  processMPU6050(origin);
}
bool start3 = true;
void mode3() {
  //多段位置模式实现，但是不能实时根据pid纠正速度、加速度
  if (start3) {
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

    displacement = 375;  // 第2段位移
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

    displacement = 0;  // 第3段位移
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
    node.writeSingleRegister(0x2333, 0);
    delay(50);

    node.writeSingleRegister(0x2300, 2);
    delay(50);
    start3 = false;
  }
}


//-------MODE1 pid转180------------
// 传感器模块：读取速度并计算PID调整值
void processSensors(float &modifiedSpeed, float &modifiedPos) {
  float targetSpeed, targetAcceleration, targetPosition;

  currentSpeed = getSpeed();
  currentAcceleration = getAcc();
  currentPos = (getPosition() - OriginPos);
  float currentTime = (millis() - offset) / 1000.0;  // 当前时间（秒）


  if (useTrapezoidalProfile) {
    trapezoidal.updateProfile(currentTime, targetSpeed, targetAcceleration, targetPosition);
  } else {
    sCurve.updateProfile(currentTime, targetSpeed, targetAcceleration, targetPosition);
  }

  // Serial.print("currentTime:");
  // Serial.print(currentTime);
  // Serial.print(" currentSpeed:");
  // Serial.print(currentSpeed);
  // Serial.print(" currentPos:");
  // Serial.println(currentPos);
  // Serial.print("Target Speed: ");
  // Serial.print(targetSpeed);
  // Serial.print(" Target Pos: ");
  // Serial.println(targetPosition);


  // 使用PID计算修正值
  if (useTrapezoidalProfile) {
    modifiedSpeed = pidSpeedT.compute(targetSpeed, currentSpeed);
    modifiedPos = pidPosT.compute(targetPosition, currentPos);
  } else {
    modifiedSpeed = pidSpeedS.compute(targetSpeed, currentSpeed);
    modifiedPos = pidPosS.compute(targetPosition, currentPos);
  }
}
void processControl() {
  //速度模式
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


  // 位置模式
  // node.writeSingleRegister(0x2109, 1);
  // delay(50);
  // node.writeSingleRegister(0x2310, 2);
  // delay(50);
  // node.writeSingleRegister(0x2311, 0);
  // delay(50);
  // node.writeSingleRegister(0x2314, 1);
  // delay(50);
  // node.writeSingleRegister(0x2315, 1);
  // delay(50);
  // int32_t displacement = 500;
  // node.setTransmitBuffer(1, lowWord(displacement));
  // node.setTransmitBuffer(0, highWord(displacement));
  // node.writeMultipleRegisters(0x2320, 2);
  // delay(50);
  // node.writeSingleRegister(0x2321, 30);
  // delay(50);
  // node.writeSingleRegister(0x2322, 10);  //第一段
  // delay(50);
  // node.writeSingleRegister(0x2323, 10);
  // delay(50);
}
// 控制模块：根据传入的修改后的速度和加速度设置Modbus寄存器
void regulateControl(float modifiedSpeed, float modifiedPos) {
  modifiedSpeed += currentSpeed;
  // delay(10);
  // int32_t displacement = modifiedPos;
  // node.setTransmitBuffer(1, lowWord(displacement));
  // node.setTransmitBuffer(0, highWord(displacement));
  // node.writeMultipleRegisters(0x2320, 2);
  delay(10);
  node.writeSingleRegister(0x2390, modifiedSpeed);
  delay(10);
  // node.writeSingleRegister(0x2316, 0);
  // delay(10);
  // node.writeSingleRegister(0x2316, 1);
  // delay(10);
  // 打印输出
  // Serial.print("Modified Speed: ");
  // Serial.println(modifiedSpeed);
  // Serial.print("Modified Acceleration: ");
  // Serial.println(modifiedPos);
}
void stopServo() {
  node.writeSingleRegister(0x2300, 1);
  delay(50);
}

float getPosition() {
  delay(10);
  uint8_t result = node.readHoldingRegisters(0x6064, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t highWord = node.getResponseBuffer(0);  // 高16位
    uint16_t lowWord = node.getResponseBuffer(1);   // 低16位
    int32_t data = ((int32_t)highWord << 16) | lowWord;
    // Serial.print("data:");
    // Serial.println(data);
    return data;
    //node.clearResponseBuffer();
  }
  return 0;
}
float getSpeed() {
  delay(10);
  uint8_t result = node.readHoldingRegisters(0x606C, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t highWord = node.getResponseBuffer(0);  // 高16位
    uint16_t lowWord = node.getResponseBuffer(1);   // 低16位
    int32_t data = ((int32_t)highWord << 16) | lowWord;
    return data;
  }
  return 0;
}
float getAcc() {
  float speed1 = getSpeed();
  float speed2 = getSpeed();
  return (speed2 - speed1) * (10 / (60 * 0.01));
}


//----------MODE2 保持平衡---------------
void initTorqueMode() {
  node.writeSingleRegister(0x2109, 1);
  delay(50);
  node.writeSingleRegister(0x2310, 0);
  delay(50);
  node.writeSingleRegister(0x2311, 0);
  delay(50);
  node.writeSingleRegister(0x2312, 0);
  delay(50);
  int32_t displacement = 0;
  node.setTransmitBuffer(1, lowWord(displacement));
  node.setTransmitBuffer(0, highWord(displacement));
  node.writeMultipleRegisters(0x2320, 2);
  delay(50);
  node.writeSingleRegister(0x2321, 0);
  delay(50);
  node.writeSingleRegister(0x2322, 30);
  delay(50);
  node.writeSingleRegister(0x2323, 30);
  delay(50);
  node.writeSingleRegister(0x2316, 0);
  delay(50);
  node.writeSingleRegister(0x2316, 1);
  delay(50);
  node.writeSingleRegister(0x2300, 2);
  delay(1000);
}
void processMPU6050(float &origin) {
  // int16_t ax, ay, az, gx, gy, gz;
  // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float pulsesPerDegree = ONE_ROLL / 360.0;

  // 采用 arctan2(accelY, accelZ) 计算倾角，单位转换为度
  mpu.update();
  float pitch = mpu.getAngleX();

  int error = origin - pitch;

  int regulateSpeed = pidPosmode2.compute(origin, pitch) * pulsesPerDegree;

  float targetPositionPulFloat = error * pulsesPerDegree;
  currentPos = -OriginPos + getPosition();
  currentSpeed = getSpeed();
  float regulatePos = pidSpeedmode2.compute(regulateSpeed, currentSpeed);
  delay(10);

  int32_t posCmdPul = (int32_t)(regulatePos);
  if (abs(error) < 2) posCmdPul = 0;
  node.setTransmitBuffer(1, lowWord(posCmdPul));
  node.setTransmitBuffer(0, highWord(posCmdPul));
  node.writeMultipleRegisters(0x2320, 2);  // 写入0x2320及后续寄存器（共2个寄存器）
  delay(10);
  node.writeSingleRegister(0x2321, 100);
  delay(10);
  node.writeSingleRegister(0x2316, 0);
  delay(10);
  node.writeSingleRegister(0x2316, 1);
  delay(10);
  origin = pitch;

  // 调试输出
  Serial.print("MPU6050 Pitch: ");
  Serial.print(pitch);
  Serial.print(" °, Error: ");
  Serial.print(error);
  Serial.print(" °, targetPositionPulFloat: ");
  Serial.print(targetPositionPulFloat);
  Serial.print(" pul, Pos Command: ");
  Serial.print(regulatePos);
  Serial.print(" pul, Speed Command: ");
  Serial.println(regulateSpeed);
  Serial.print(" rpm,currentPos :");
  Serial.print(currentPos);
  Serial.print(" pul,currentSpeed :");
  Serial.print(currentSpeed);
  Serial.println(" rpm");
}



// 显示模块：更新 OLED 显示屏内容
// 主显示函数
void processDisplay() {
  display.clearDisplay();

  // 1. 获取传感器数据
  // int16_t ax, ay, az;
  // mpu.getAcceleration(&ax, &ay, &az);
  // float AccXangle = atan2((float)ay, sqrt(pow((float)ax, 2) + pow((float)az, 2))) * 180 / PI;
  updateSpeedHistory(currentSpeed);

  // 第2行：速度
  char buf[32];
  sprintf(buf, "Speed: %5.1f rpm", currentSpeed);
  display.setCursor(0, 10);
  display.print(buf);


  // 3. 中间图表区 (速度曲线)
  const int graphY = 32;       // 图表起始Y坐标
  const int graphHeight = 56;  // 图表高度
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
    Serial.print("Received!");
    mode = 2;
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





//----------测试部分------------
void debug() {
  // 发送调试命令给伺服电机
  sendServoCommand();
  delay(50);  // 等待伺服电机响应
  readServoResponse();
}

/**
 * 发送伺服电机调试命令
 */
void sendServoCommand() {

  node.writeSingleRegister(0x2109, 2);  // 设置模式
  delay(50);
  node.writeSingleRegister(0x2380, 0);
  delay(50);
  node.writeSingleRegister(0x2385, 10);
  delay(50);
  node.writeSingleRegister(0x2390, 100);  // 设定目标速度
  delay(50);
  node.writeSingleRegister(0x2391, 20);
  delay(50);
  node.writeSingleRegister(0x2300, 2);
  delay(50);
  uint8_t result = node.writeSingleRegister(0x2384, 0);  // 设定加速度为 10 rps/s
  delay(50);

  if (result == node.ku8MBSuccess) {
    Serial.println("Command sent successfully.");
  } else {
    Serial.print("Command failed with error code: ");
    Serial.println(result);
  }
  float pos = getPosition();
  delay(10);
  float speed = getSpeed();
  delay(10);
  float acc = getAcc();
  Serial.print("Pos:");
  Serial.print(pos);
  Serial.print(" Speed:");
  Serial.print(speed);
  Serial.print(" Acc: ");
  Serial.println(acc);
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
