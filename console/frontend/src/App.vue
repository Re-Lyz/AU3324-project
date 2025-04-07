<template>
  <div class="app-container">
    <div class="content-left">
      <h1>云台控制台</h1>
      
      <!-- 通信状态显示 -->
      <div class="connection-status">
        <div class="status" :class="{ connected: commStatus.isConnected }">
          {{ commStatus.isConnected ? '已连接' : '未连接' }}
        </div>
        <div class="mode-info">
          <span v-if="commStatus.isConnected">
            当前模式: {{ commStatus.currentMode === 'bluetooth' ? '蓝牙' : 'WiFi' }}
          </span>
          <span v-else>正在尝试连接...</span>
        </div>
        <div class="available-modes">
          <span>蓝牙: {{ commStatus.bluetoothAvailable ? '可用' : '不可用' }}</span>
          <span>WiFi: {{ commStatus.wifiAvailable ? '可用' : '不可用' }}</span>
        </div>
      </div>
      
      <AngleChart ref="chart" />
      
      <!-- 显示传感器数据 -->
      <div class="sensor-data">
        <h3>实时传感器数据 ({{ sensorData.source === 'bluetooth' ? '蓝牙' : 'WiFi' }})</h3>
        <p>X轴角度: {{ sensorData.angleX.toFixed(2) }}°</p>
        <p>加速度: 
          X:{{ sensorData.accX }}, 
          Y:{{ sensorData.accY }}, 
          Z:{{ sensorData.accZ }}
        </p>
        <p>舵机位置: {{ sensorData.servoPosition }}°</p>
        <p>更新时间: {{ sensorData.timestamp || '暂无数据' }}</p>
      </div>
    </div>

    <div class="content-right">
      <div class="panel-wrapper">
        <h2>控制面板</h2>
        <button class="panel-button" @click="sendRotateRequest">旋转云台</button>
        
        <!-- 通信模式选择 -->
        <div class="mode-selector">
          <h3>优先通信模式</h3>
          <div class="radio-group">
            <label>
              <input 
                type="radio" 
                value="bluetooth" 
                v-model="preferredMode"
                @change="setPreferredMode"
              >
              蓝牙优先
            </label>
            <label>
              <input 
                type="radio" 
                value="wifi" 
                v-model="preferredMode"
                @change="setPreferredMode"
              >
              WiFi优先
            </label>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { ref, onMounted, onBeforeUnmount } from 'vue';
import AngleChart from './components/AngleChart.vue';

export default {
  components: { AngleChart },
  setup() {
    const SERVER_PORT = 8081;
    let socket = null;
    
    // 通信状态
    const commStatus = ref({
      preferredMode: 'bluetooth',
      currentMode: null,
      isConnected: false,
      bluetoothAvailable: false,
      wifiAvailable: false,
      lastUpdate: null
    });
    
    // 传感器数据
    const sensorData = ref({
      angleX: 0,
      accX: 0,
      accY: 0,
      accZ: 0,
      servoPosition: 0,
      timestamp: null,
      source: null
    });
    
    // 图表引用
    const chart = ref(null);
    
    // 优先模式选择
    const preferredMode = ref('bluetooth');

    /**
     * WebSocket连接
     */
    const connectWebSocket = () => {
      socket = new WebSocket(`ws://localhost:${SERVER_PORT}`);

      socket.onopen = () => {
        console.log('Connected to WebSocket server');
      };

      socket.onmessage = (event) => {
        try {
          const { type, data, source } = JSON.parse(event.data);
          
          if (type === 'sensor_data') {
            // 更新传感器数据
            sensorData.value = {
              angleX: parseFloat(data.angleX),
              accX: parseInt(data.accX),
              accY: parseInt(data.accY),
              accZ: parseInt(data.accZ),
              servoPosition: parseInt(data.servoPosition),
              timestamp: data.lastUpdate,
              source: source
            };
            
            // 更新图表
            if (chart.value) {
              chart.value.updateChart(sensorData.value.angleX);
            }
            
          } else if (type === 'comm_status') {
            // 更新通信状态
            commStatus.value = {
              preferredMode: data.preferredMode,
              currentMode: data.currentMode,
              isConnected: data.isConnected,
              bluetoothAvailable: data.bluetoothAvailable,
              wifiAvailable: data.wifiAvailable,
              lastUpdate: data.lastUpdate
            };
            
            // 同步优先模式选择
            preferredMode.value = data.preferredMode;
          }
        } catch (e) {
          console.error('Error parsing WebSocket message:', e);
        }
      };

      socket.onclose = () => {
        commStatus.value.isConnected = false;
        console.log('Disconnected from WebSocket server');
      };

      socket.onerror = (error) => {
        console.error('WebSocket error:', error);
      };
    };

    /**
     * 发送旋转云台请求
     */
    const sendRotateRequest = async () => {
      try {
        const response = await fetch(`http://localhost:${SERVER_PORT}/api/rotate`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          }
        });
        
        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.message || 'Request failed');
        }
        
        const result = await response.json();
        console.log(`旋转命令已通过${result.mode === 'bluetooth' ? '蓝牙' : 'WiFi'}发送`);
      } catch (error) {
        console.error('Error:', error);
        alert(`操作失败: ${error.message}`);
      }
    };
    
    /**
     * 设置优先通信模式
     */
    const setPreferredMode = async () => {
      try {
        const response = await fetch(`http://localhost:${SERVER_PORT}/api/set_preferred_mode`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify({ mode: preferredMode.value })
        });
        
        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.message || 'Failed to set preferred mode');
        }
        
        console.log(`优先通信模式已设置为: ${preferredMode.value}`);
      } catch (error) {
        console.error('Error:', error);
        alert(`设置失败: ${error.message}`);
      }
    };

    onMounted(() => {
      connectWebSocket();
    });

    onBeforeUnmount(() => {
      if (socket) {
        socket.close();
      }
    });

    return {
      commStatus,
      sensorData,
      chart,
      preferredMode,
      sendRotateRequest,
      setPreferredMode
    };
  }
};
</script>

<style>
/* ======================================== 主容器样式 ======================================== */
.app-container {
  display: flex;
  justify-content: space-between;
  margin: 0 auto;
  padding: 20px;
  font-family: Arial, sans-serif;
}

.content-left {
  flex: 2; 
  padding: 10px;
  min-width: 0; /* 防止内容溢出 */
  width: 1200px;
}

.content-right {
  flex: 1;
  padding: 10px;
  min-width: 300px;
}

h1 {
  text-align: center;
  color: #333;
  margin-bottom: 20px;
}

h2, h3 {
  color: #333;
  margin-top: 0;
}

/* ======================================== 连接状态样式 ======================================== */
.connection-status {
  background-color: #f5f5f5;
  border-radius: 8px;
  padding: 15px;
  margin-bottom: 20px;
}

.status {
  text-align: center;
  padding: 10px;
  margin-bottom: 10px;
  border-radius: 4px;
  font-weight: bold;
  background-color: #ffebee;
  color: #c62828;
}

.status.connected {
  background-color: #e8f5e9;
  color: #2e7d32;
}

.mode-info {
  text-align: center;
  margin: 10px 0;
  font-size: 14px;
}

.available-modes {
  display: flex;
  justify-content: space-around;
  font-size: 13px;
  color: #666;
}

/* ======================================== 传感器数据样式 ======================================== */
.sensor-data {
  background-color: #f9f9f9;
  border-radius: 8px;
  padding: 15px;
  margin-top: 20px;
}

.sensor-data p {
  margin: 8px 0;
  font-size: 14px;
}

/* ======================================== 控制面板样式 ======================================== */
.panel-wrapper {
  background-color: #f9f9f9;
  border-radius: 8px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  padding: 20px;
  height: 100%;
  box-sizing: border-box;
}

.panel-button {
  display: block;
  width: 100%;
  padding: 12px;
  font-size: 16px;
  cursor: pointer;
  text-align: center;
  color: #fff;
  background-color: #42a5f5;
  border: none;
  border-radius: 6px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
  transition: all 0.3s;
  margin: 20px 0;
}

.panel-button:hover {
  background-color: #1e88e5;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
}

.panel-button:active {
  transform: translateY(2px);
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.2);
}

/* ======================================== 模式选择器样式 ======================================== */
.mode-selector {
  margin-top: 30px;
}

.radio-group {
  display: flex;
  flex-direction: column;
  gap: 10px;
  margin-top: 10px;
}

.radio-group label {
  display: flex;
  align-items: center;
  cursor: pointer;
  font-size: 14px;
}

.radio-group input[type="radio"] {
  margin-right: 8px;
}

/* ======================================== 响应式设计 ======================================== */
@media (max-width: 768px) {
  .app-container {
    flex-direction: column;
  }
  
  .content-left, .content-right {
    width: 100%;
  }
}
</style>