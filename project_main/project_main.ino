#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <BluetoothSerial.h>


// ------------------------ 全局变量与常量 ------------------------
// OLED 屏幕参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//端口设置
#define RS485_DE_RE_PIN 4
#define RS485_BAUD 9600

// 创建 MPU6050 对象
MPU6050 mpu;
BluetoothSerial SerialBT;  // 蓝牙串口实例

// 定义其他需要的全局变量（例如 RS485 通信、伺服电机参数等）
int servoPosition = 0;  // 当前伺服电机位置（示例变量）
bool btConnected = false;  // BT连接状态标志

// ------------------------ 模块函数声明 ------------------------

// 1. 硬件初始化模块：负责初始化所有硬件接口与外设
void initHardware();

// 2. 通信模块：负责与伺服电机之间通过 RS485 进行数据传输
void processRS485Communication();

// 3. 传感器模块：负责读取 MPU6050 等传感器数据
void processSensors();

// 4. 控制算法模块：实现闭环控制算法，包括180°旋转控制、S曲线/梯形速度规划以及水平保持
void processControl();

// 5. 显示模块：更新 OLED 显示屏上的实时数据（如速度曲线、角度信息等）
void processDisplay();

// 6. 无线通信模块：实现 Wi-Fi 或蓝牙的远程数据传输和控制（预留扩展接口）
void processWireless();

// 7. 系统调试与任务调度模块（可选）：用于调试输出和任务调度

// ------------------------ setup() 函数 ------------------------
void setup() {
  // 初始化硬件
  initHardware();

  // 蓝牙初始化
  if(!SerialBT.begin("ESP32_Servo")) {  // 设备名称
    Serial.println("蓝牙初始化失败!");
  } else {
    Serial.println("蓝牙已就绪，名称: ESP32_Servo");
    SerialBT.setPin("1234", 4);  // 设置配对密码
  }
}

// ------------------------ loop() 函数 ------------------------
void loop() {
  // 1. 读取传感器数据
  processSensors();

  // 2. 执行闭环控制算法
  processControl();

  // 3. 处理 RS485 通信
  processRS485Communication();

  // 4. 更新 OLED 显示数据
  processDisplay();

  // 5. 执行无线通信任务（如Wi-Fi或蓝牙）
  processWireless();

  // 可根据需要添加调试输出、日志记录或任务调度控制

  // 1. 发送调试命令给伺服电机
  sendServoCommand();

  // 2. 稍作等待后读取伺服电机的响应
  delay(50);  // 等待伺服电机响应
  readServoResponse();

  // 3. 每隔一段时间发送一次调试命令
  delay(1000);

  delay(10);  // 简单延时，实际项目中可使用定时器实现精确调度
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
    Serial.println("OLED 初始化失败");
    while (1)
      ;  // 初始化失败则停留在此
  }
  display.clearDisplay();
  display.display();

  // 初始化 MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 连接失败");
  } else {
    Serial.println("MPU6050 连接成功");
  }

  // 初始化其他硬件模块（例如 RS485 模块、伺服电机的控制接口等）
  Serial2.begin(RS485_BAUD);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  Serial.println("伺服电机调试程序启动");
}

// 通信模块：处理 RS485 与伺服电机的通信
void processRS485Communication() {
  // 示例代码：发送控制命令、接收反馈数据
  // 可以通过 Serial、硬件串口或软件串口来实现 RS485 通信
  // 这里仅打印信息作为示例
  Serial.println("RS485 通信处理中...");
  // 实际实现时，可调用 RS485 库函数进行数据收发
}

// 传感器模块：读取 MPU6050 或其他传感器数据
void processSensors() {
  // 示例：读取 MPU6050 加速度与陀螺仪数据
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // 打印或存储数据供后续控制算法使用
  Serial.print("加速度: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);
}

// 控制算法模块：实现闭环控制及速度规划
void processControl() {
  // 示例：根据传感器数据和当前伺服状态计算目标位置或速度
  // 此处仅模拟控制算法逻辑，实际应设计PID或其他闭环控制
  servoPosition = (servoPosition + 1) % 180;  // 模拟180°内运动
  Serial.print("目标位置: ");
  Serial.println(servoPosition);

  // 根据设计要求生成 S 曲线或梯形速度曲线
  // 调整 PID 参数等
}

// 显示模块：更新 OLED 显示屏内容
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
 * 这里以发送一个示例字符串命令为例，实际应用中请根据具体协议构造二进制数据包
 */
void sendServoCommand() {
  // 切换 RS485 模块至发送模式
  digitalWrite(RS485_DE_RE_PIN, HIGH);

  // 示例命令：这里假设发送 "SET_POS:90" 表示将电机设置到90°位置
  const char* debugCmd = "SET_POS:90\n";
  Serial.print("发送命令：");
  Serial.println(debugCmd);

  // 发送命令到 RS485 总线
  Serial2.print(debugCmd);

  // 等待数据发送完成后（或使用 Serial2.flush()）再切换回接收模式
  delay(10);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

/**
 * 读取伺服电机的响应数据并在串口监视器输出
 */
void readServoResponse() {
  if (Serial2.available()) {
    Serial.println("接收到伺服电机响应：");
    // 循环读取所有数据
    while (Serial2.available()) {
      char c = Serial2.read();
      Serial.print(c);
    }
    Serial.println();  // 换行
  } else {
    Serial.println("未接收到响应。");
  }
}
