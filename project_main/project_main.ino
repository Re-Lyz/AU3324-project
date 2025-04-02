#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <ModbusMaster.h>
#include <BluetoothSerial.h>
#include <math.h>

// ------------------------ 全局变量与常量 ------------------------
// OLED 屏幕参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RS485 通信波特率
#define RS485_BAUD 115200

MPU6050 mpu;
ModbusMaster node;
BluetoothSerial SerialBT;  // 声明蓝牙串口实例

// 定义其他全局变量（例如伺服电机参数等）
int servoPosition = 0;     // 当前伺服电机位置（示例变量）
bool btConnected = false;  // 蓝牙连接状态标志
bool debugmode = true;

// 定义目标速度和加速度全局常量（用于PID计算）
const float targetSpeed = 500.0;       // 目标速度，单位 rpm
const float targetAcceleration = 10.0;   // 目标加减速，单位 rps/s

// ------------------------ 模块函数声明 ------------------------
void initHardware();
void processRS485Communication();
void processSensors(float &modifiedSpeed, float &modifiedAcceleration);
void processControl(float modifiedSpeed, float modifiedAcceleration);
void processDisplay();
void processWireless();
void sendServoCommand();
void readServoResponse();
void handleCommand(String cmd);

// PID控制器类
class PID {
public:
  // 构造函数，初始化PID常数
  PID(float p, float i, float d) : kp(p), ki(i), kd(d), prevError(0), integral(0) {}

  // 计算PID控制输出
  float compute(float setpoint, float input) {
    float error = setpoint - input;           // 计算误差
    float pTerm = kp * error;                   // 比例项
    integral += error;                          // 积分累加
    float iTerm = ki * integral;                // 积分项
    float dTerm = kd * (error - prevError);       // 微分项
    float output = pTerm + iTerm + dTerm;         // 计算PID输出
    prevError = error;                          // 更新前一误差
    return output;
  }

private:
  float kp;      // 比例常数
  float ki;      // 积分常数
  float kd;      // 微分常数
  float prevError; // 上一次的误差
  float integral;  // 积分值
};

// PID控制器实例
PID pidSpeed(1.0, 0.1, 0.05);         // 用于速度控制的PID
PID pidAcceleration(1.0, 0.1, 0.05);    // 用于加减速控制的PID

// ------------------------ setup() 函数 ------------------------
void setup() {
  initHardware();

  uint8_t result = node.readHoldingRegisters(0x606C, 2);
  if (result == node.ku8MBSuccess) {
    Serial.println("Read registers successfully:");
    Serial.print("Register 0: ");
    Serial.println(node.getResponseBuffer(0));
    Serial.print("Register 1: ");
    Serial.println(node.getResponseBuffer(1));
  } else {
    Serial.print("Error reading registers, error code: ");
    Serial.println(result);
  }

  // 设置伺服电机为位置模式，并初始化一些参数（寄存器地址参考文档）
  node.writeSingleRegister(0x2109, 1);  // 设置为位置模式
  delay(100);
  node.writeSingleRegister(0x2310, 3);
  delay(100);
  node.writeSingleRegister(0x2311, 1);  
  delay(100);
  node.writeSingleRegister(0x2320, 10000); // 设定目标位置为 10000 脉冲
  delay(100);
  node.writeSingleRegister(0x2321, 500);  // 设定目标速度为 500 rpm
  delay(100);
  node.writeSingleRegister(0x2322, 10);   // 设定加速度为 10 rps/s
  delay(100);
  node.writeSingleRegister(0x2323, 10);   // 设定减速度为 10 rps/s
  delay(100);
  node.writeSingleRegister(0x2316, 0);    // 运动触发信号设定为 ON
  delay(100);
  node.writeSingleRegister(0x2316, 1);    // 运动触发信号设定为 ON
  delay(100);
}

// ------------------------ loop() 函数 ------------------------
void loop() {
  if (debugmode) {
    // 发送调试命令给伺服电机
    sendServoCommand();
    delay(50);  // 等待伺服电机响应
    readServoResponse();
    delay(1000);  // 每隔一段时间发送一次调试命令
  } 
  float modifiedSpeed, modifiedAcceleration;
  processSensors(modifiedSpeed, modifiedAcceleration);
  processControl(modifiedSpeed, modifiedAcceleration);
  processRS485Communication();
  processDisplay();
  processWireless();
  
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
    Serial.println("OLED failed");
    while (1);
  }
  display.clearDisplay();
  display.display();

  // 初始化 MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 connection successful");
  }

  // 初始化 RS485 通信：ESP32 的 Serial2 设置为 RX->D16, TX->D17
  Serial2.begin(RS485_BAUD, SERIAL_8N1, 17, 16);
  node.begin(01, Serial2);
  Serial.println("Modbus RTU initialization complete.");

  // 蓝牙初始化
  if (!SerialBT.begin("ESP32_Servo")) {  // 设备名称
    Serial.println("蓝牙初始化失败!");
  } else {
    Serial.println("蓝牙已就绪，名称: ESP32_Servo");
    SerialBT.setPin("1234", 4);  // 设置配对密码
  }
} // end of initHardware

// RS485通信处理模块
void processRS485Communication() {
  Serial.println("RS485 communication processing...");
}

// 传感器模块：读取 MPU6050 数据并计算PID调整值
void processSensors(float &modifiedSpeed, float &modifiedAcceleration) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 当前速度和加速度（假设由传感器获得，这里使用陀螺仪和加速度计的数据）
  float currentSpeed = gx;         // 以陀螺仪 x 轴数据作为当前速度
  float currentAcceleration = ax;    // 以加速度计 x 轴数据作为当前加速度

  // 使用PID计算修正值
  modifiedSpeed = pidSpeed.compute(targetSpeed, currentSpeed);
  modifiedAcceleration = pidAcceleration.compute(targetAcceleration, currentAcceleration);

  Serial.print("Acceleration: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);
}

// 控制模块：根据传入的修改后的速度和加速度设置Modbus寄存器
void processControl(float modifiedSpeed, float modifiedAcceleration) {
  // 设置为速度模式（寄存器 0x2109 设置为速度模式值 2）
  node.writeSingleRegister(0x2109, 2);
  delay(100);

  // 设置加减速速率（寄存器 0x2385），单位为 0.1 rps/s，乘以 10 转换
  node.writeSingleRegister(0x2385, modifiedAcceleration * 10);
  delay(100);

  // 设置目标速度（寄存器 0x2390）
  node.writeSingleRegister(0x2390, modifiedSpeed);
  delay(100);

  // 打印输出
  Serial.print("Target Position: ");
  Serial.println(servoPosition);
  Serial.print("Modified Speed: ");
  Serial.println(modifiedSpeed);
  Serial.print("Modified Acceleration: ");
  Serial.println(modifiedAcceleration);

  // 更新目标位置，保持循环
  servoPosition = (servoPosition + 1) % 180;
}

// 显示模块：更新 OLED 显示内容
void processDisplay() {
  display.clearDisplay();
 
  int16_t ax, ay, az;
  float AccXangle;
  mpu.getAcceleration(&ax, &ay, &az);
 
  // 计算 X 轴角度
  AccXangle = atan((float)ay / sqrt(pow((float)ax, 2) + pow((float)az, 2))) * 180 / PI;
 
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Angle X: ");
  display.println(AccXangle);
 
  // 显示蓝牙连接状态
  display.setCursor(0, 56);
  display.print("BT: ");
  display.print(btConnected ? "Connected" : "Available");
 
  // 绘制指示线
  int x_center = SCREEN_WIDTH / 2;
  int y_center = SCREEN_HEIGHT / 2;
  int line_length = 30;
  float angle_rad = AccXangle * PI / 180.0;
  int x_end = x_center + line_length * sin(angle_rad);
  int y_end = y_center - line_length * cos(angle_rad);
  display.drawLine(x_center, y_center, x_end, y_end, SSD1306_WHITE);
 
  display.display();
}

// 无线通信模块
void processWireless() {
  Serial.println("无线通信任务处理中...");
  bool currentConnected = SerialBT.hasClient();
  if (currentConnected != btConnected) {
    btConnected = currentConnected;
    Serial.println(btConnected ? "蓝牙已连接" : "蓝牙已断开");
  }
 
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();
    Serial.print("==========> 蓝牙消息：");
    Serial.println(command);
    handleCommand(command);
  }
}
 
// 消费蓝牙消息（使用 if/else 处理 String 命令）
void handleCommand(String cmd) {
  if (cmd.equals("ROTATE")) {
    // TODO：实现电机旋转180度
  } else {
    Serial.print("==========> 蓝牙命令不存在：");
    Serial.println(cmd);
  }
}
 
/**
 * 发送伺服电机调试命令
 */
void sendServoCommand() {

  node.writeSingleRegister(0x2109, 2);  // 设置为位置模式
  delay(50);
  node.writeSingleRegister(0x2380, 2); 
  delay(50);
  node.writeSingleRegister(0x2385, 100); 
  delay(50);
  node.writeSingleRegister(0x2390, 500);  // 设定目标速度为 500 rpm
  delay(50);
  node.writeSingleRegister(0x2384, 1);   // 设定加速度为 10 rps/s
  delay(5000);
  uint8_t result = node.writeSingleRegister(0x2384, 0);   // 设定加速度为 10 rps/s
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

/**
 * 更新梯形速度曲线下的目标速度和加速度
 * 参数:
 *  - t: 当前时间（秒）
 *  - t_acc: 加速阶段时间（秒）
 *  - t_const: 匀速阶段时间（秒）
 *  - t_dec: 减速阶段时间（秒）
 *  - maxSpeed: 目标最大速度（单位：例如度/秒或脉冲/秒）
 *  - a: 加速度（正值）
 *  - d: 减速度（正值）
 * 输出:
 *  - targetSpeed: 当前目标速度
 *  - targetAcceleration: 当前目标加速度
 */
void updateTrapezoidalProfile(float t, float t_acc, float t_const, float t_dec, float maxSpeed, float a, float d,
                              float &targetSpeed, float &targetAcceleration) {
  if (t < t_acc) {
    // 加速阶段：线性加速
    targetAcceleration = a;
    targetSpeed = a * t;
  } else if (t < t_acc + t_const) {
    // 匀速阶段：保持最大速度
    targetAcceleration = 0;
    targetSpeed = maxSpeed;
  } else if (t < t_acc + t_const + t_dec) {
    // 减速阶段：线性减速
    float t_dec_phase = t - t_acc - t_const;
    targetAcceleration = -d;
    targetSpeed = maxSpeed - d * t_dec_phase;
  } else {
    // 运动结束：速度和加速度均为0
    targetAcceleration = 0;
    targetSpeed = 0;
  }
}

/**
 * 更新 S 曲线加速阶段下的目标速度和加速度
 * 参数:
 *  - t: 当前时间（秒），t 范围在 [0, t_acc_total]
 *  - t_r: 加速斜坡上升时间（秒），在此阶段加速度从 0 增加到 A_max
 *  - t_const: 恒定加速度阶段时间（秒）
 *  - t_r_down: 加速斜坡下降时间（秒），在此阶段加速度从 A_max 降低到 0
 *  - A_max: 最大加速度
 * 输出:
 *  - targetSpeed: 当前目标速度（积分计算得到）
 *  - targetAcceleration: 当前目标加速度
 */
void updateSCurveProfile(float t, float t_r, float t_const, float t_r_down, float A_max,
                         float &targetSpeed, float &targetAcceleration) {
  float t_acc_total = t_r + t_const + t_r_down;
  if (t < t_r) {
    // 斜坡上升阶段：加速度从 0 线性增加到 A_max
    targetAcceleration = A_max * (t / t_r);
    // 积分得到速度：速度 = 1/2 * A_max * (t^2 / t_r)
    targetSpeed = 0.5 * A_max * (t * t / t_r);
  } else if (t < t_r + t_const) {
    // 恒定加速度阶段：加速度为 A_max
    targetAcceleration = A_max;
    // 先计算上升阶段累积的速度
    float speed_ramp = 0.5 * A_max * t_r;
    // 再加上当前恒加速阶段的速度增量
    float t_const_phase = t - t_r;
    targetSpeed = speed_ramp + A_max * t_const_phase;
  } else if (t < t_acc_total) {
    // 斜坡下降阶段：加速度从 A_max 线性降低到 0
    float t_down = t - t_r - t_const;
    targetAcceleration = A_max * (1 - t_down / t_r_down);
    // 计算上面两段累积的速度
    float speed_ramp = 0.5 * A_max * t_r;
    float speed_const = A_max * t_const;
    // 本阶段速度积分（近似计算）
    float speed_ramp_down = A_max * t_down - 0.5 * A_max * (t_down * t_down / t_r_down);
    targetSpeed = speed_ramp + speed_const + speed_ramp_down;
  } else {
    targetAcceleration = 0;
    // 超出加速阶段时间后，目标速度可设为一个预定值或保持上一个值
    // 这里简化处理为保持加速阶段最后的速度
    float speed_ramp = 0.5 * A_max * t_r;
    float speed_const = A_max * t_const;
    float speed_ramp_down = A_max * t_r_down - 0.5 * A_max * t_r_down;  // = 0.5*A_max*t_r_down
    targetSpeed = speed_ramp + speed_const + speed_ramp_down;
  }
}


