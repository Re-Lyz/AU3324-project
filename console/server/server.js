const express = require('express');
const WebSocket = require('ws');
const bodyParser = require('body-parser');
const cors = require('cors');
const BluetoothSerialPort = require('bluetooth-serial-port').BluetoothSerialPort;
const net = require('net');

const app = express();
const PORT = 8081;
const wss = new WebSocket.Server({ noServer: true });

// 通信状态
const commState = {
  preferredMode: 'bluetooth', // 优先使用蓝牙
  currentMode: null,          // 当前实际使用的通信模式
  isConnected: false,         // 当前连接状态
  lastUpdate: null,           // 最后更新时间
};

// 蓝牙配置
const bluetoothState = {
  btSerial: new BluetoothSerialPort(),
  isAvailable: false,
  sensorData: {
    angleX: 0,
    accX: 0,
    accY: 0,
    accZ: 0,
    servoPosition: 0,
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
    angleX: 0,
    accX: 0,
    accY: 0,
    accZ: 0,
    servoPosition: 0,
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

// ====================== 通信模式管理 ======================
function updateConnectionStatus() {
  // 检查蓝牙状态
  bluetoothState.btSerial.isOpen((isOpen) => {
    bluetoothState.isAvailable = isOpen;
    
    // 检查WiFi状态
    const wifiAvailable = wifiState.socket && !wifiState.socket.destroyed && wifiState.isAvailable;
    
    // 确定当前通信模式
    let newMode = null;
    if (commState.preferredMode === 'bluetooth' && bluetoothState.isAvailable) {
      newMode = 'bluetooth';
    } else if (wifiAvailable) {
      newMode = 'wifi';
    }
    
    // 状态变化处理
    if (newMode !== commState.currentMode) {
      commState.currentMode = newMode;
      commState.isConnected = newMode !== null;
      commState.lastUpdate = new Date().toISOString();
      
      // 模式切换时停止另一模式的数据采集
      if (newMode === 'bluetooth') {
        stopWifiDataCollection();
        startBluetoothDataCollection();
      } else if (newMode === 'wifi') {
        stopBluetoothDataCollection();
        startWifiDataCollection();
      } else {
        stopBluetoothDataCollection();
        stopWifiDataCollection();
      }
      
      broadcastCommStatus();
    }
  });
}

function broadcastCommStatus() {
  broadcastToClients({
    type: 'comm_status',
    data: {
      preferredMode: commState.preferredMode,
      currentMode: commState.currentMode,
      isConnected: commState.isConnected,
      lastUpdate: commState.lastUpdate,
      bluetoothAvailable: bluetoothState.isAvailable,
      wifiAvailable: wifiState.isAvailable
    }
  });
}

function broadcastSensorData() {
  const data = commState.currentMode === 'bluetooth' 
    ? bluetoothState.sensorData 
    : wifiState.sensorData;
  
  broadcastToClients({
    type: 'sensor_data',
    data: data,
    source: commState.currentMode
  });
}

// ====================== WebSocket处理 ======================
wss.on('connection', (ws) => {
    console.log('客户端已连接');
    
    // 发送初始状态和数据
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'comm_status',
            data: {
                preferredMode: commState.preferredMode,
                currentMode: commState.currentMode,
                isConnected: commState.isConnected,
                lastUpdate: commState.lastUpdate,
                bluetoothAvailable: bluetoothState.isAvailable,
                wifiAvailable: wifiState.isAvailable
            }
        }));
        
        const currentData = commState.currentMode === 'bluetooth' 
          ? bluetoothState.sensorData 
          : wifiState.sensorData;
        
        ws.send(JSON.stringify({
            type: 'sensor_data',
            data: currentData,
            source: commState.currentMode
        }));
    }
    
    ws.on('close', () => {
        console.log('客户端已断开连接');
    });
});

// ====================== 蓝牙通信模块 ======================
function startBluetoothDataCollection() {
    stopBluetoothDataCollection();
    
    const dataRequestCommand = 'GET_DATA\n';
    
    bluetoothState.dataInterval = setInterval(() => {
        if (bluetoothState.isAvailable) {
            bluetoothState.btSerial.write(Buffer.from(dataRequestCommand), (err) => {
                if (err) {
                    console.error('蓝牙数据请求发送失败:', err);
                    bluetoothState.isAvailable = false;
                    updateConnectionStatus();
                }
            });
        }
    }, DATA_INTERVAL);
}

function stopBluetoothDataCollection() {
    if (bluetoothState.dataInterval) {
        clearInterval(bluetoothState.dataInterval);
        bluetoothState.dataInterval = null;
    }
}

bluetoothState.btSerial.on('data', (buffer) => {
    const rawData = buffer.toString('utf-8').trim();
    processBluetoothData(rawData);
});

bluetoothState.btSerial.on('closed', () => {
    console.log('蓝牙连接已断开');
    bluetoothState.isAvailable = false;
    updateConnectionStatus();
});

function processBluetoothData(rawData) {
    try {
        if (rawData.startsWith('DATA:')) {
            const parts = rawData.substring(5).split(',');
            if (parts.length >= 5) {
                bluetoothState.sensorData = {
                    angleX: parseFloat(parts[0]),
                    accX: parseInt(parts[1]),
                    accY: parseInt(parts[2]),
                    accZ: parseInt(parts[3]),
                    servoPosition: parseInt(parts[4]),
                    lastUpdate: new Date().toISOString()
                };
                
                if (commState.currentMode === 'bluetooth') {
                    broadcastSensorData();
                }
            }
        }
    } catch (e) {
        console.error('蓝牙数据解析错误:', e);
    }
}

// ====================== WiFi通信模块 ======================
function initWifiConnection() {
  if (wifiState.socket) {
    wifiState.socket.destroy();
  }

  wifiState.socket = new net.Socket();
  
  wifiState.socket.on('connect', () => {
    console.log('WiFi连接已建立');
    wifiState.isAvailable = true;
    clearTimeout(wifiState.reconnectTimer);
    updateConnectionStatus();
  });

  wifiState.socket.on('data', (data) => {
    const rawData = data.toString().trim();
    processWifiData(rawData);
  });

  wifiState.socket.on('error', (err) => {
    console.error('WiFi连接错误:', err.message);
    handleWifiDisconnect();
  });

  wifiState.socket.on('close', () => {
    console.log('WiFi连接已关闭');
    handleWifiDisconnect();
  });

  console.log('正在尝试连接WiFi...');
  wifiState.socket.connect(WIFI_CONFIG.port, WIFI_CONFIG.host);
}

function handleWifiDisconnect() {
  if (wifiState.isAvailable) {
    wifiState.isAvailable = false;
    updateConnectionStatus();
  }
  
  if (!wifiState.reconnectTimer) {
    wifiState.reconnectTimer = setTimeout(() => {
      wifiState.reconnectTimer = null;
      initWifiConnection();
    }, WIFI_CONFIG.reconnectInterval);
  }
}

function processWifiData(rawData) {
  try {
    if (rawData.startsWith('DATA:')) {
      const parts = rawData.substring(5).split(',');
      if (parts.length >= 5) {
        wifiState.sensorData = {
          angleX: parseFloat(parts[0]),
          accX: parseInt(parts[1]),
          accY: parseInt(parts[2]),
          accZ: parseInt(parts[3]),
          servoPosition: parseInt(parts[4]),
          lastUpdate: new Date().toISOString()
        };
        
        if (commState.currentMode === 'wifi') {
          broadcastSensorData();
        }
      }
    }
  } catch (e) {
    console.error('WiFi数据解析错误:', e);
  }
}

function startWifiDataCollection() {
  stopWifiDataCollection();
  
  const dataRequestCommand = 'GET_DATA\n';
  
  wifiState.dataInterval = setInterval(() => {
    if (wifiState.isAvailable) {
      wifiState.socket.write(dataRequestCommand, (err) => {
        if (err) {
          console.error('WiFi数据请求发送失败:', err);
          wifiState.isAvailable = false;
          updateConnectionStatus();
        }
      });
    }
  }, DATA_INTERVAL);
}

function stopWifiDataCollection() {
  if (wifiState.dataInterval) {
    clearInterval(wifiState.dataInterval);
    wifiState.dataInterval = null;
  }
}

// ====================== 统一API接口 ======================
app.post('/api/rotate', (req, res) => {
    console.log('==========> 【POST】旋转180度');
    
    if (!commState.isConnected) {
        console.log('无可用连接');
        return res.status(400).send({ 
            message: '未连接到设备。请确保蓝牙或WiFi已连接。' 
        });
    }

    const command = 'ROTATE\n';
    
    if (commState.currentMode === 'bluetooth') {
        bluetoothState.btSerial.write(Buffer.from(command), (err) => {
            if (err) {
                console.error('蓝牙命令发送错误:', err);
                bluetoothState.isAvailable = false;
                updateConnectionStatus();
                return res.status(500).send({ 
                    message: '蓝牙命令发送失败，尝试使用WiFi...' 
                });
            }
            console.log('旋转命令已通过蓝牙发送');
            res.status(200).send({ 
                message: '旋转命令发送成功',
                mode: 'bluetooth'
            });
        });
    } else {
        wifiState.socket.write(command, (err) => {
            if (err) {
                console.error('WiFi命令发送错误:', err);
                wifiState.isAvailable = false;
                updateConnectionStatus();
                return res.status(500).send({ 
                    message: 'WiFi命令发送失败' 
                });
            }
            console.log('旋转命令已通过WiFi发送');
            res.status(200).send({ 
                message: '旋转命令发送成功',
                mode: 'wifi'
            });
        });
    }
});

// 设置优先通信模式
app.post('/api/set_preferred_mode', (req, res) => {
    const { mode } = req.body;
    if (mode === 'bluetooth' || mode === 'wifi') {
        commState.preferredMode = mode;
        updateConnectionStatus();
        res.status(200).send({
            message: `优先通信模式已设置为: ${mode}`,
            currentMode: commState.currentMode
        });
    } else {
        res.status(400).send({
            message: '无效的模式，请使用"bluetooth"或"wifi"'
        });
    }
});

// ====================== 辅助函数 ======================
function broadcastToClients(message) {
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify(message));
        }
    });
}

// ====================== 服务器启动 ======================
const server = app.listen(PORT, () => {
    console.log(`服务器已启动，监听端口 ${PORT}`);
    
    // 初始化蓝牙连接检测
    bluetoothState.btSerial.on('finished', () => {
        bluetoothState.isAvailable = false;
        updateConnectionStatus();
    });
    
    // 初始化WiFi连接
    initWifiConnection();
    
    // 定期检查蓝牙状态
    setInterval(updateConnectionStatus, 3000);
});

server.on('upgrade', (request, socket, head) => {
    wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request);
    });
});