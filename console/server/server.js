const express = require('express');
const WebSocket = require('ws');
const bodyParser = require('body-parser');
const cors = require('cors');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const net = require('net');

const app = express();
const PORT = 8081;
const wss = new WebSocket.Server({ noServer: true });

// 串口配置
const SERIAL_PORT = 'COM5';
const BAUD_RATE = 57600;

// 通信状态
const commState = {
  preferredMode: 'bluetooth', // 优先使用蓝牙
  currentMode: null,          // 当前实际使用的通信模式
  isConnected: false,         // 当前连接状态
  lastUpdate: null,           // 最后更新时间
};

// 系统状态
const systemState = {
  isConnected: false,
  port: null,
  parser: null,
  sensorData: {
    speed: 0,
    acceleration: 0,
    lastUpdate: null
  },
  dataInterval: null
};

// WiFi配置
const WIFI_CONFIG = {
  host: '192.168.4.1',
  port: 8080,
  reconnectInterval: 5000
};

const wifiState = {
  socket: null,
  isAvailable: false,
  sensorData: {
    speed: 0,
    acceleration: 0,
    lastUpdate: null
  },
  reconnectTimer: null,
  dataInterval: null
};

const DATA_INTERVAL = 500;

app.use(bodyParser.json());
app.use(cors({
  origin: 'http://localhost:8080',
  methods: ['GET', 'POST'],
  allowedHeaders: ['Content-Type']
}));

/**
 * 初始化串口连接
 */
function initSerialPort() {
  // 如果已有连接，先关闭
  if (systemState.port && systemState.port.isOpen) {
    systemState.port.close();
  }

  systemState.port = new SerialPort({
    path: SERIAL_PORT,
    baudRate: BAUD_RATE,
    autoOpen: false
  });

  // 创建解析器
  systemState.parser = systemState.port.pipe(new ReadlineParser({ delimiter: '\n' }));

  // 事件处理
  systemState.port.on('open', () => {
    console.log(`Serial port ${SERIAL_PORT} opened`);
    updateConnectionStatus(true);
    startDataCollection();
  });

  systemState.port.on('close', () => {
    console.log(`Serial port ${SERIAL_PORT} closed`);
    updateConnectionStatus(false);
  });

  systemState.port.on('error', (err) => {
    console.error('Serial port error:', err);
    updateConnectionStatus(false);
  });

  // 数据接收处理
  systemState.parser.on('data', (data) => {
    processIncomingData(data);
  });

  // 尝试打开端口
  systemState.port.open((err) => {
    if (err) {
      console.error('Failed to open serial port:', err.message);
      updateConnectionStatus(false);
    }
  });
}

/**
 * 处理接收到的数据
 */
function processIncomingData(rawData) {
  console.log('Received raw data:', rawData);
  
  try {
    if (rawData.startsWith('DATA:')) {
      const parts = rawData.substring(5).split(':');
      if (parts.length == 2) {
        systemState.sensorData = {
          speed: parseFloat(parts[0]),
          acceleration: parseFloat(parts[1]),
          lastUpdate: new Date().toISOString()
        };
        
        console.log("==========> Data: " + JSON.stringify(systemState.sensorData));
        // 广播给所有客户端
        broadcastToClients({
          type: 'sensor_data',
          data: systemState.sensorData
        });
      }
    }
  } catch (e) {
    console.error('Data parsing error:', e);
  }
}

/**
 * 更新连接状态并通知客户端
 */
function updateConnectionStatus(isConnected) {
  if (systemState.isConnected !== isConnected) {
    systemState.isConnected = isConnected;
    
    if (!isConnected) {
      stopDataCollection();
    }
    
    broadcastToClients({
      type: 'status',
      data: {
        isConnected: isConnected,
        lastUpdate: new Date().toISOString()
      }
    });
    
    // 如果断开连接，尝试重新连接
    if (!isConnected) {
      setTimeout(initSerialPort, 5000); // 5秒后尝试重连
    }
  }
}

/**
 * 开始数据采集
 */
function startDataCollection() {
  stopDataCollection(); // 确保先停止之前的
  
  systemState.dataInterval = setInterval(() => {
    if (systemState.isConnected && systemState.port?.isOpen) {
      const dataRequestCommand = 'GET_DATA\n';
      systemState.port.write(dataRequestCommand, (err) => {
        if (err) {
          console.error('Failed to send data request:', err);
          updateConnectionStatus(false);
        }
      });
    }
  }, DATA_INTERVAL);
  
  console.log('Started data collection');
}

/**
 * 停止数据采集
 */
function stopDataCollection() {
  if (systemState.dataInterval) {
    clearInterval(systemState.dataInterval);
    systemState.dataInterval = null;
    console.log('Stopped data collection');
  }
}

/**
 * WebSocket连接处理
 */
wss.on('connection', (ws) => {
  console.log('Client connected');
  
  // 发送初始状态和数据
  ws.send(JSON.stringify({
    type: 'status',
    data: {
      isConnected: systemState.isConnected,
      lastUpdate: new Date().toISOString()
    }
  }));
  
  ws.send(JSON.stringify({
    type: 'sensor_data',
    data: systemState.sensorData
  }));
  
  ws.on('close', () => {
    console.log('Client disconnected');
  });
});

/**
 * 广播消息给所有客户端
 */
function broadcastToClients(message) {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(message));
    }
  });
}

/**
 * API端点 - 旋转180度
 */
app.post('/api/rotate', (req, res) => {
  console.log('==========> 【POST】旋转180度');
  
  if (!systemState.isConnected || !systemState.port?.isOpen) {
    return res.status(400).json({ 
      message: 'Device is not connected. Please check the connection.' 
    });
  }

  const command = 'ROTATE\n';
  systemState.port.write(command, (err) => {
    if (err) {
      console.error('Write error:', err);
      updateConnectionStatus(false);
      return res.status(500).json({ 
        message: 'Failed to send command. Connection may be lost.' 
      });
    }
    console.log('Rotation command sent');
    res.status(200).json({ message: 'Rotation command sent successfully' });
  });
});

// 启动服务器
const server = app.listen(PORT, () => {
  console.log(`Server is listening on port ${PORT}`);
  initSerialPort(); // 初始化串口连接
});

// WebSocket服务器升级处理
server.on('upgrade', (request, socket, head) => {
  wss.handleUpgrade(request, socket, head, (ws) => {
    wss.emit('connection', ws, request);
  });
});

// 优雅关闭处理
process.on('SIGINT', () => {
  console.log('Shutting down server...');
  
  // 关闭串口连接
  if (systemState.port?.isOpen) {
    systemState.port.close();
  }
  
  // 关闭WebSocket服务器
  wss.close();
  
  // 关闭HTTP服务器
  server.close(() => {
    process.exit(0);
  });
});