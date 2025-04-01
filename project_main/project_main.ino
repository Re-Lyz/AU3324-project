#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <ModbusMaster.h>
#include <BluetoothSerial.h>

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

debugmode=True;

// 定义其他全局变量（例如伺服电机参数等）
int servoPosition = 0;   // 当前伺服电机位置（示例变量）
bool btConnected = false;  // BT连接状态标志

// ------------------------ 模块函数声明 ------------------------
void initHardware();
void processRS485Communication();
void processSensors();
void processControl();
void processDisplay();
void processWireless();
void sendServoCommand();
void readServoResponse();

// PID控制器类
class PID {
public:
  // 构造函数，初始化PID常数
  PID(float p, float i, float d) : kp(p), ki(i), kd(d), prevError(0), integral(0) {}

  // 计算PID控制输出
  float compute(float setpoint, float input) {
    // 计算误差
    float error = setpoint - input;

    float pTerm = kp * error;    // 比例项
    integral += error;
    float iTerm = ki * integral;    // 积分项
    float dTerm = kd * (error - prevError);    // 微分项

    // 计算PID输出
    float output = pTerm + iTerm + dTerm;
    // 更新前一误差
    prevError = error;
    return output;
  }

private:
  float kp;  // 比例常数
  float ki;  // 积分常数
  float kd;  // 微分常数

  float prevError;  // 上一次的误差
  float integral;   // 积分值
};

// PID控制器实例
PID pidSpeed(1.0, 0.1, 0.05);  // PID参数：比例常数、积分常数、微分常数
PID pidAcceleration(1.0, 0.1, 0.05);  // PID参数：比例常数、积分常数、微分常数

// ------------------------ setup() 函数 ------------------------
void setup() {
  initHardware();
  node.begin(1, Serial2);
  Serial.println("Modbus RTU initialization complete.");

    // Set the servo motor to position mode (address 2109h)
  node.writeSingleRegister(0x2109, 1);  // Set to position mode
  delay(100); // Delay for the mode to apply
  node.writeSingleRegister(0x2320, 10000); // Set target position to 10000 pulses
  delay(100);
  node.writeSingleRegister(0x2321, 500);  // Set target speed to 500 rpm
  delay(100);
  node.writeSingleRegister(0x2322, 10);   // Set acceleration to 10 rps/s
  delay(100);
  node.writeSingleRegister(0x2323, 10);   // Set deceleration to 10 rps/s
  delay(100);
  node.writeSingleRegister(0x2300, 2);    // Set trigger to ON
  delay(100);

}

// ------------------------ loop() 函数 ------------------------
void loop() {
  if(debugmode){
    // 发送调试命令给伺服电机
    sendServoCommand();
    delay(50); // 等待伺服电机响应
    readServoResponse();
    delay(1000); // 每隔一段时间发送一次调试命令
  }
  else{
    float modifiedSpeed, modifiedAcceleration;
    processSensors(modifiedSpeed, modifiedAcceleration);
    processControl(modifiedSpeed, modifiedAcceleration);
    processRS485Communication();
    processDisplay();
    processWireless();
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

  // 初始化 RS485 通信
  // 将 ESP32 的 Serial2 设置为：RX 连接到 D16，TX 连接到 D17
  Serial2.begin(RS485_BAUD, SERIAL_8N1, 16, 17);

  Serial.println("start testing");

    // 蓝牙初始化
  if(!SerialBT.begin("ESP32_Servo")) {  // 设备名称
    Serial.println("蓝牙初始化失败!");
  } else {
    Serial.println("蓝牙已就绪，名称: ESP32_Servo");
    SerialBT.setPin("1234", 4);  // 设置配对密码
  }


// 通信模块：示例仅打印调试信息
void processRS485Communication() {
  Serial.println("RS485 communication processing...");
}

// 传感器模块：读取 MPU6050 数据
void processSensors(float &modifiedSpeed, float &modifiedAcceleration) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 当前速度和加速度 (假设由传感器获取，这里用虚拟值)
  float currentSpeed = gx;  // 以陀螺仪的x轴数据为当前速度
  float currentAcceleration = ax;  // 以加速度计的x轴数据为当前加速度

  // 使用PID控制器计算修改后的速度和加速度
  modifiedSpeed = pidSpeed.compute(targetSpeed, currentSpeed);  // 计算修正后的速度
  modifiedAcceleration = pidAcceleration.compute(targetAcceleration, currentAcceleration);  // 计算修正后的加速度

  Serial.print("accleration: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);

}

void processControl(float modifiedSpeed, float modifiedAcceleration) {
  // 设置为速度模式 (2109h 寄存器设置为速度模式)
  node.writeSingleRegister(0x2109, 2);  // 设置为速度模式
  delay(100);  // 延时让设置生效

  // 设置加减速速率 (2385h 寄存器设置加减速)
  // 传入 modifiedAcceleration 参数，单位是 0.1 rps/s，因此乘以 10 得到实际的加减速速率
  node.writeSingleRegister(0x2385, modifiedAcceleration * 10);  // 设置加减速速率为传入的 modifiedAcceleration
  delay(100);

  // 设置目标速度 (2390h 寄存器设置目标速度)
  node.writeSingleRegister(0x2390, modifiedSpeed);  // 设置目标速度为传入的 modifiedSpeed
  delay(100);

  // 输出当前的目标位置和速度
  Serial.print("Target Position: ");
  Serial.println(servoPosition);
  Serial.print("Modified Speed: ");
  Serial.println(modifiedSpeed);
  Serial.print("Modified Acceleration: ");
  Serial.println(modifiedAcceleration);

  // 更新目标位置，保持位置的循环调整
  servoPosition = (servoPosition + 1) % 180;
}

// 显示模块：更新 OLED 显示内容
void processDisplay() {
   // 清空屏幕
   display.clearDisplay();
 
   int16_t ax, ay, az;
   float AccXangle;
   mpu.getAcceleration(&ax, &ay, &az);
 
   // 计算 X 轴角度
   AccXangle = atan((float)ay / sqrt(pow((float)ax, 2) + pow((float)az, 2))) * 180 / PI;
 
   // 显示角度
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
   int angle_rad = AccXangle * PI / 180;
   int x_end = x_center + line_length * sin(angle_rad);
   int y_end = y_center - line_length * cos(angle_rad);
 
   display.drawLine(x_center, y_center, x_end, y_end, SSD1306_WHITE);
 
   // 更新显示
   display.display();
 }

// 无线通信模块
 void processWireless() {
   // 示例：无线模块任务处理，可包括连接状态检测、数据收发等
   Serial.println("无线通信任务处理中...");
   // 实际实现时，调用 Wi-Fi 或蓝牙库函数进行数据处理
   // 检查连接状态变化
   bool currentConnected = SerialBT.hasClient();
   if(currentConnected != btConnected) {
     btConnected = currentConnected;
     Serial.println(btConnected ? "蓝牙已连接" : "蓝牙已断开");
   }
 
   // 处理收到的命令
   if(SerialBT.available()) {
     String command = SerialBT.readStringUntil('\n');
     command.trim();
     Serial.print("==========> 蓝牙消息：");
     Serial.println(command);
     handleCommand(command);
   }
 }
 
 /**
  * 消费蓝牙消息
  */
 void handleCommand(String cmd) {
   // TODO
 }

/**
 * 发送伺服电机调试命令
 * 示例：发送 "SET_POS:90" 命令给伺服电机
 */
void sendServoCommand() {
  // 清空内部缓存
  node.clearTransmitBuffer();
  // 设置内部缓存数据：
  // 第 0 个寄存器为高 16 位（0x0000）
  node.setTransmitBuffer(0, 0x0000);
  // 第 1 个寄存器为低 16 位（0x00FA，250 的十六进制表示）
  node.setTransmitBuffer(1, 0x00FA);
  
  // 发送写多个寄存器命令，寄存器起始地址为 0x2320，写入 2 个寄存器
  uint8_t result = node.writeMultipleRegisters((uint16_t)0x2320, (uint16_t)2);
  
  if (result == node.ku8MBSuccess) {
    Serial.println("Command sent successfully.");
  } else {
    Serial.print("Command failed with error code: ");
    Serial.println(result);
  }
}

void handleCommand(String cmd) {
   // TODO
   switch(cmd) {
     case "ROTATE":
       // TODO：实现电机旋转180度
       break;
     default:
       Serial.print("==========> 蓝牙命令不存在：");
       Serial.println(cmd);
   }
 }

/**
 * 读取伺服电机响应数据并输出到串口监视器
 */
void readServoResponse() {
  if (Serial2.available()) {
    Serial.println("receive server response:");
    while (Serial2.available()) {
      char c = Serial2.read();
      Serial.print(c);
    }
    Serial.println();
  } else {
    Serial.println("no response");
  }
}
