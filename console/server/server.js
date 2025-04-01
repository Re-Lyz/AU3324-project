const express = require('express');
const WebSocket = require('ws');
const bodyParser = require('body-parser');
const cors = require('cors');
const BluetoothSerialPort = require('bluetooth-serial-port').BluetoothSerialPort;

const app = express();
const PORT = 8081;
const wss = new WebSocket.Server({ noServer: true });

// 蓝牙状态
let bluetoothState = {
  isConnected: false,
  btSerial: new BluetoothSerialPort()
};

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

    // 发送当前蓝牙状态
    updateBluetoothStatus();
    
    // 定期检查连接状态
    const statusInterval = setInterval(updateBluetoothStatus, 5000);

    ws.on('close', () => {
        clearInterval(statusInterval);
        console.log('Client disconnected');
    });
});

/**
 * 更新蓝牙连接状态并通知客户端
 */
function updateBluetoothStatus() {
    // 检查当前连接状态
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
        }
    });
}

/**
 * 旋转180度
 */
app.post('/api/rotate', (req, res) => {
    console.log('==========> 【POST】旋转180度');
    updateBluetoothStatus(); // 先更新状态
    
    if (!bluetoothState.isConnected) {
        return res.status(400).send({ 
            message: 'No ESP32 device is connected. Please connect manually first.' 
        });
    }

    const command = 'ROTATE_180\n';
    
    bluetoothState.btSerial.write(Buffer.from(command), (err) => {
        if (err) {
            console.error('Write error:', err);
            updateBluetoothStatus(); // 写入失败可能意味着连接已断开
            return res.status(500).send({ 
                message: 'Failed to send command. Connection may be lost.' 
            });
        }
        console.log('Rotation command sent to ESP32');
        res.status(200).send({ message: 'Rotation command sent successfully' });
    });
});

// ====================== 蓝牙通信事件处理 ======================

// 被动接收数据（仅当已连接时）
bluetoothState.btSerial.on('data', (buffer) => {
    const data = buffer.toString('utf-8').trim();
    console.log('Received data:', data);
    
    broadcastToClients({
        type: 'bluetooth_data',
        data: data
    });
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
    
    // 初始状态检查
    updateBluetoothStatus();
});

server.on('upgrade', (request, socket, head) => {
    wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request);
    });
});