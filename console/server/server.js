const express = require('express');
const WebSocket = require('ws');
const bodyParser = require('body-parser');
const cors = require('cors');
const BluetoothSerialPort = require('bluetooth-serial-port').BluetoothSerialPort;

const app = express();
const PORT = 8081;
const wss = new WebSocket.Server({ noServer: true });

// 蓝牙状态和数据
let bluetoothState = {
  isConnected: false,
  btSerial: new BluetoothSerialPort(),
  sensorData: {
    angleX: 0,
    accX: 0,
    accY: 0,
    accZ: 0,
    servoPosition: 0,
    lastUpdate: null
  }
};

// 数据采集间隔(ms)
const DATA_INTERVAL = 500;

app.use(bodyParser.json());
app.use(cors({
    origin: 'http://localhost:8080', 
    methods: ['GET', 'POST'], 
    allowedHeaders: ['Content-Type']
}));

/**
 * WebSocket连接
 */
wss.on('connection', (ws) => {
    console.log('Client connected');

    // 发送当前蓝牙状态和最新数据
    sendInitialData(ws);
    ws.on('close', () => {
        clearInterval(statusInterval);
        console.log('Client disconnected');
    });
});

/**
 * 初始化数据发送
 */
function sendInitialData(ws) {
    updateBluetoothStatus();
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'status',
            data: {
                isConnected: bluetoothState.isConnected,
                lastUpdate: new Date().toISOString()
            }
        }));
        
        ws.send(JSON.stringify({
            type: 'sensor_data',
            data: bluetoothState.sensorData
        }));
    }
}

/**
 * 更新蓝牙连接状态并通知客户端
 */
function updateBluetoothStatus() {
    bluetoothState.btSerial.isOpen((isOpen) => {
        if (bluetoothState.isConnected !== isOpen) {
            bluetoothState.isConnected = isOpen;
            broadcastToClients({
                type: 'status',
                data: {
                    isConnected: isOpen,
                    lastUpdate: new Date().toISOString()
                }
            });
            
            // 连接状态变化时，重置或开始数据采集
            if (isOpen) {
                startDataCollection();
            } else {
                stopDataCollection();
            }
        }
    });
}

/**
 * 开始数据采集
 */
function startDataCollection() {
    // 清除之前的定时器（如果有）
    stopDataCollection();
    
    // 请求数据命令（根据您的ESP32代码调整）
    const dataRequestCommand = 'GET_DATA\n';
    
    // 设置定时请求数据
    bluetoothState.dataInterval = setInterval(() => {
        if (bluetoothState.isConnected) {
            bluetoothState.btSerial.write(Buffer.from(dataRequestCommand), (err) => {
                if (err) {
                    console.error('Failed to send data request:', err);
                    updateBluetoothStatus();
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
    if (bluetoothState.dataInterval) {
        clearInterval(bluetoothState.dataInterval);
        bluetoothState.dataInterval = null;
        console.log('Stopped data collection');
    }
}

/**
 * 旋转180度
 */
app.post('/api/rotate', (req, res) => {
    console.log('==========> 【POST】旋转180度');
    updateBluetoothStatus();
    
    if (!bluetoothState.isConnected) {
        return res.status(400).send({ 
            message: 'No ESP32 device is connected. Please connect manually first.' 
        });
    }

    const command = 'ROTATE\n';  
    bluetoothState.btSerial.write(Buffer.from(command), (err) => {
        if (err) {
            console.error('Write error:', err);
            updateBluetoothStatus();
            return res.status(500).send({ 
                message: 'Failed to send command. Connection may be lost.' 
            });
        }
        console.log('Rotation command sent to ESP32');
        res.status(200).send({ message: 'Rotation command sent successfully' });
    });
});

// ====================== 蓝牙通信事件处理 ======================

// 接收数据
bluetoothState.btSerial.on('data', (buffer) => {
    const rawData = buffer.toString('utf-8').trim();
    console.log('Received raw data:', rawData);
    
    try {
        // 解析数据（根据ESP32发送的实际格式调整）
        // 预期格式: "DATA:angleX,ax,ay,az,servoPos"
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
                
                // 广播给所有客户端
                broadcastToClients({
                    type: 'sensor_data',
                    data: bluetoothState.sensorData
                });
            }
        }
    } catch (e) {
        console.error('Data parsing error:', e);
    }
});

// 连接断开事件
bluetoothState.btSerial.on('closed', () => {
    console.log('Connection closed by remote device');
    updateBluetoothStatus();
});

// ====================== 辅助函数 ======================

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

// ==========================================================

const server = app.listen(PORT, () => {
    console.log(`Server is listening on port ${PORT}`);
    updateBluetoothStatus();
});

server.on('upgrade', (request, socket, head) => {
    wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request);
    });
});