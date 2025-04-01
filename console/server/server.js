const express = require('express');
const WebSocket = require('ws');
const bodyParser = require('body-parser');
const cors = require('cors');
const app = express();
const PORT = 3001;

app.use(bodyParser.json());
app.use(cors({
    origin: 'http://localhost:5173', 
    methods: ['GET', 'POST'], 
    allowedHeaders: ['Content-Type']
}));

// 创建WebSocket服务器
const wss = new WebSocket.Server({ noServer: true });

wss.on('connection', (ws) => {
    console.log('Client connected');

    // 当接收到消息时
    ws.on('message', (message) => {
        const data = JSON.parse(message);
        if(data.action === 'connect') {
            // 模拟连接ESP32设备
            setTimeout(() => {
                ws.send(JSON.stringify({
                    type: 'status',
                    data: 'Connection successful'
                }));
            }, 1000);
        }
    });

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});

// 处理旋转请求的POST接口
app.post('/api/rotate', (req, res) => {
    console.log('Rotation request header:', req.header);
    console.log('Rotation request received:', req.body);

    // 在这里可以添加实际的逻辑，例如向ESP32发送命令

    // 返回成功的响应
    res.status(200).send({ message: 'Rotation request processed' });
});

// 设置HTTP服务器监听特定端口，并集成WebSocket服务器
const server = app.listen(PORT, () => {
    console.log(`Server is listening on port ${PORT}`);
});

server.on('upgrade', (request, socket, head) => {
    wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request);
    });
});