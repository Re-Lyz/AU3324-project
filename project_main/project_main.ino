#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <ModbusMaster.h>

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

// 定义其他全局变量（例如伺服电机参数等）
int servoPosition = 0;   // 当前伺服电机位置（示例变量）

// ------------------------ 模块函数声明 ------------------------
void initHardware();
void processRS485Communication();
void processSensors();
void processControl();
void processDisplay();
void processWireless();
void sendServoCommand();
void readServoResponse();

// ------------------------ setup() 函数 ------------------------
void setup() {
  initHardware();
  node.begin(1, Serial2);
  Serial.println("Modbus RTU initialization complete.");
}

// ------------------------ loop() 函数 ------------------------
void loop() {
  processSensors();
  processControl();
  processRS485Communication();
  processDisplay();
  processWireless();

  // 发送调试命令给伺服电机
  sendServoCommand();
  delay(50); // 等待伺服电机响应
  readServoResponse();
  delay(1000); // 每隔一段时间发送一次调试命令
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
}

// 通信模块：示例仅打印调试信息
void processRS485Communication() {
  Serial.println("RS485 communication processing...");
}

// 传感器模块：读取 MPU6050 数据
void processSensors() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("accleration: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);
}

// 控制算法模块：示例计算目标位置
void processControl() {
  servoPosition = (servoPosition + 1) % 180;
  Serial.print("target position: ");
  Serial.println(servoPosition);
}

// 显示模块：更新 OLED 显示内容
void processDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Servo Pos: ");
  display.println(servoPosition);
  display.display();
}

// 无线通信模块：示例仅打印调试信息
void processWireless() {
  Serial.println("wifi processing...");
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
