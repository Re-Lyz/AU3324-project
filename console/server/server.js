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
  preferredMode: 'bluetooth', 
  currentMode: null,          
  isConnected: false,       
  bluetoothAvailable: false,
  wifiAvailable: false,
  lastUpdate: null,   
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

// ==================================================== 蓝牙通信 ====================================================

/**
 * 初始化串口连接
 */
function initSerialPort() {
  if (systemState.port && systemState.port.isOpen) {
    systemState.port.close();
  }

  systemState.port = new SerialPort({
    path: SERIAL_PORT,
    baudRate: BAUD_RATE,
    autoOpen: false
  });

  systemState.parser = systemState.port.pipe(new ReadlineParser({ delimiter: '\n' }));
  systemState.port.on('open', () => {
    console.log(`Serial port ${SERIAL_PORT} opened`);
    commState.bluetoothAvailable = true;
    updateConnectionStatus(true);
    updateCommStatus();
    startDataCollection();
  });

  systemState.port.on('close', () => {
    console.log(`Serial port ${SERIAL_PORT} closed`);
    commState.bluetoothAvailable = false;
    updateConnectionStatus(false);
    updateCommStatus();
  });

  systemState.port.on('error', (err) => {
    console.error('Serial port error:', err);
    commState.bluetoothAvailable = false;
    updateConnectionStatus(false);
    updateCommStatus();
  });

  systemState.parser.on('data', (data) => {
    processIncomingData(data);
  });

  systemState.port.open((err) => {
    if (err) {
      console.error('Failed to open serial port:', err.message);
      commState.bluetoothAvailable = false;
      updateConnectionStatus(false);
      updateCommStatus();
    }
  });
}

/**
 * 处理接收到的蓝牙数据
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
        
        console.log("==========> Bluetooth Data: " + JSON.stringify(systemState.sensorData));
        
        // 如果蓝牙是首选模式或WiFi不可用，使用蓝牙数据
        if (commState.preferredMode === 'bluetooth' || !wifiState.isAvailable) {
          broadcastToClients({
            type: 'sensor_data',
            data: systemState.sensorData,
            source: 'bluetooth'
          });
        }
      }
    }
  } catch (e) {
    console.error('Bluetooth data parsing error:', e);
  }
}

/**
 * 更新蓝牙连接状态
 */
function updateConnectionStatus(isConnected) {
  if (systemState.isConnected !== isConnected) {
    systemState.isConnected = isConnected;
    
    if (!isConnected) {
      stopDataCollection();
    }
    
    updateCommStatus();
    
    // 如果断开连接，尝试重新连接
    if (!isConnected) {
      setTimeout(initSerialPort, 5000);
    }
  }
}

/**
 * 开始蓝牙数据采集
 */
function startDataCollection() {
  stopDataCollection();
  
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
  
  console.log('Started Bluetooth data collection');
}

/**
 * 停止蓝牙数据采集
 */
function stopDataCollection() {
  if (systemState.dataInterval) {
    clearInterval(systemState.dataInterval);
    systemState.dataInterval = null;
    console.log('Stopped Bluetooth data collection');
  }
}

// ==================================================== WiFi通信 ====================================================

/**
 * 初始化WiFi连接
 */
function initWifiConnection() {
  // 关闭现有连接
  if (wifiState.socket) {
    wifiState.socket.destroy();
    clearTimeout(wifiState.reconnectTimer);
  }

  wifiState.socket = new net.Socket();
  
  wifiState.socket.on('connect', () => {
    console.log('WiFi connected to', WIFI_CONFIG.host);
    wifiState.isAvailable = true;
    commState.wifiAvailable = true;
    updateCommStatus();
    startWifiDataCollection();
  });

  wifiState.socket.on('data', (data) => {
    processWifiData(data.toString());
  });

  wifiState.socket.on('error', (err) => {
    console.error('WiFi connection error:', err);
    handleWifiDisconnect();
  });

  wifiState.socket.on('close', () => {
    console.log('WiFi connection closed');
    handleWifiDisconnect();
  });

  // 尝试连接
  wifiState.socket.connect(WIFI_CONFIG.port, WIFI_CONFIG.host);
}

/**
 * 处理WiFi断开连接
 */
function handleWifiDisconnect() {
  wifiState.isAvailable = false;
  commState.wifiAvailable = false;
  stopWifiDataCollection();
  updateCommStatus();
  
  // 尝试重新连接
  wifiState.reconnectTimer = setTimeout(() => {
    initWifiConnection();
  }, WIFI_CONFIG.reconnectInterval);
}

/**
 * 处理接收到的WiFi数据
 */
function processWifiData(rawData) {
  try {
    if (rawData.startsWith('DATA:')) {
      const parts = rawData.substring(5).split(':');
      if (parts.length == 2) {
        wifiState.sensorData = {
          speed: parseFloat(parts[0]),
          acceleration: parseFloat(parts[1]),
          lastUpdate: new Date().toISOString()
        };
        
        console.log("==========> WiFi Data: " + JSON.stringify(wifiState.sensorData));
        
        // 如果WiFi是首选模式或蓝牙不可用，使用WiFi数据
        if (commState.preferredMode === 'wifi' || !systemState.isConnected) {
          broadcastToClients({
            type: 'sensor_data',
            data: wifiState.sensorData,
            source: 'wifi'
          });
        }
      }
    }
  } catch (e) {
    console.error('WiFi data parsing error:', e);
  }
}

/**
 * 开始WiFi数据采集
 */
function startWifiDataCollection() {
  stopWifiDataCollection();
  
  wifiState.dataInterval = setInterval(() => {
    if (wifiState.socket && wifiState.isAvailable) {
      wifiState.socket.write('GET_DATA\n');
    }
  }, DATA_INTERVAL);
  
  console.log('Started WiFi data collection');
}

/**
 * 停止WiFi数据采集
 */
function stopWifiDataCollection() {
  if (wifiState.dataInterval) {
    clearInterval(wifiState.dataInterval);
    wifiState.dataInterval = null;
    console.log('Stopped WiFi data collection');
  }
}

// ==================================================== 通信状态管理 ====================================================

/**
 * 更新通信状态并通知客户端
 */
function updateCommStatus() {
  const newStatus = {
    preferredMode: commState.preferredMode,
    currentMode: determineCurrentMode(),
    isConnected: systemState.isConnected || wifiState.isAvailable,
    bluetoothAvailable: systemState.isConnected,
    wifiAvailable: wifiState.isAvailable,
    lastUpdate: new Date().toISOString()
  };
  
  // 更新全局状态
  Object.assign(commState, newStatus);
  
  broadcastToClients({
    type: 'comm_status',
    data: newStatus
  });
}

/**
 * 确定当前使用的通信模式
 */
function determineCurrentMode() {
  if (commState.preferredMode === 'bluetooth' && systemState.isConnected) {
    return 'bluetooth';
  }
  if (commState.preferredMode === 'wifi' && wifiState.isAvailable) {
    return 'wifi';
  }
  if (wifiState.isAvailable) return 'wifi';
  if (systemState.isConnected) return 'bluetooth';
  return null;
}

// ==================================================== 前后端数据传输 ====================================================

wss.on('connection', (ws) => {
  console.log('Client connected');
  
  // 发送初始状态和数据
  ws.send(JSON.stringify({
    type: 'comm_status',
    data: {
      preferredMode: commState.preferredMode,
      currentMode: commState.currentMode,
      isConnected: commState.isConnected,
      bluetoothAvailable: commState.bluetoothAvailable,
      wifiAvailable: commState.wifiAvailable,
      lastUpdate: commState.lastUpdate
    }
  }));
  
  // 发送当前数据
  const currentData = commState.currentMode === 'wifi' ? wifiState.sensorData : systemState.sensorData;
  ws.send(JSON.stringify({
    type: 'sensor_data',
    data: currentData,
    source: commState.currentMode
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

// ==================================================== API接口 ====================================================

/**
 * 设置优先通信模式
 */
app.post('/api/set_preferred_mode', (req, res) => {
  const { mode } = req.body;
  
  if (mode !== 'bluetooth' && mode !== 'wifi') {
    return res.status(400).json({ 
      message: 'Invalid mode. Must be "bluetooth" or "wifi"' 
    });
  }

  commState.preferredMode = mode;
  updateCommStatus();
  
  res.status(200).json({ 
    message: `Preferred mode set to ${mode}`,
    currentMode: commState.currentMode
  });
});

/**
 * 旋转云台
 */
app.post('/api/rotate', (req, res) => {
  if (commState.preferredMode === 'bluetooth' && systemState.isConnected) {
    // 通过蓝牙发送
    const command = 'ROTATE\n';
    systemState.port.write(command, (err) => {
      if (err) {
        updateConnectionStatus(false);
        return tryWifiFallback(res);
      }
      res.status(200).json({ 
        message: 'Rotation command sent via Bluetooth',
        mode: 'bluetooth'
      });
    });
  } else if (wifiState.isAvailable) {
    // 通过WiFi发送
    wifiState.socket.write('ROTATE\n', (err) => {
      if (err) {
        handleWifiDisconnect();
        return res.status(500).json({ 
          message: 'Failed to send command via WiFi' 
        });
      }
      res.status(200).json({ 
        message: 'Rotation command sent via WiFi',
        mode: 'wifi'
      });
    });
  } else {
    res.status(400).json({ 
      message: 'No available connection' 
    });
  }
});

/**
 * WiFi备用方案
 */
function tryWifiFallback(res) {
  if (wifiState.isAvailable) {
    wifiState.socket.write('ROTATE\n', (err) => {
      if (err) {
        handleWifiDisconnect();
        return res.status(500).json({ 
          message: 'Failed to send command via both Bluetooth and WiFi' 
        });
      }
      res.status(200).json({ 
        message: 'Rotation command sent via WiFi (Bluetooth failed)',
        mode: 'wifi'
      });
    });
  } else {
    res.status(400).json({ 
      message: 'Bluetooth failed and WiFi not available' 
    });
  }
}

// ==================================================== 服务器启动 ====================================================

const server = app.listen(PORT, () => {
  console.log(`Server is listening on port ${PORT}`);
  initSerialPort(); // 初始化蓝牙串口连接
  initWifiConnection(); // 初始化WiFi连接
});

// WebSocket服务器升级处理
server.on('upgrade', (request, socket, head) => {
  wss.handleUpgrade(request, socket, head, (ws) => {
    wss.emit('connection', ws, request);
  });
});

// 优雅关闭
process.on('SIGINT', () => {
  console.log('Shutting down server...');

  if (systemState.port?.isOpen) {
    systemState.port.close();
  }

  if (wifiState.socket) {
    wifiState.socket.destroy();
    clearTimeout(wifiState.reconnectTimer);
  }

  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.close();
    }
  });
  wss.close();

  server.close(() => {
    console.log('Server shutdown complete');
    process.exit(0);
  });

  setTimeout(() => {
    console.warn('Forcing shutdown after timeout');
    process.exit(0);
  }, 5000);
});