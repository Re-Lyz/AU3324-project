<template>
  <div class="app-container">
    <div class="content-left">
      <h1>云台控制台</h1>
      <div class="status" :class="{ connected }">
        {{ connected ? '已连接' : '未连接' }}
      </div>
      <AngleChart ref="chart" />
      
      <!-- 显示传感器数据 -->
      <div class="sensor-data">
        <h3>实时传感器数据</h3>
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
    const connected = ref(false);
    const chart = ref(null);
    const SERVER_PORT = 8081;
    let socket = null;
    let sensorData = ref({
      angleX: 0,
      accX: 0,
      accY: 0,
      accZ: 0,
      servoPosition: 0,
      timestamp: null
    });

    /**
     * WebSocket获取实时云台数据
     */
    const connectWebSocket = () => {
      socket = new WebSocket(`ws://localhost:${SERVER_PORT}`);

      socket.onopen = () => {
        console.log('Connected to WebSocket server');
      };

      socket.onmessage = (event) => {
        try {
          const { type, data } = JSON.parse(event.data);
          
          if (type === 'sensor_data') {
            // 更新传感器数据
            sensorData.value = {
              angleX: parseFloat(data.angleX),
              accX: parseInt(data.accX),
              accY: parseInt(data.accY),
              accZ: parseInt(data.accZ),
              servoPosition: parseInt(data.servoPosition),
              timestamp: data.lastUpdate
            };
            
            // 更新图表
            if (chart.value) {
              chart.value.updateChart(sensorData.value.angleX);
            }
            
          } else if (type === 'status') {
            connected.value = data.isConnected;
          } else if (type === 'error') {
            console.error('Error:', data);
          }
        } catch (e) {
          console.error('Error parsing WebSocket message:', e);
        }
      };

      socket.onclose = () => {
        connected.value = false;
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
        
        console.log('Rotation request sent successfully');
      } catch (error) {
        console.error('Error:', error);
        alert(`操作失败: ${error.message}`);
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
      connected,
      sensorData,
      chart,
      sendRotateRequest
    };
  }
};

</script>

<style>
/* ======================================== 主容器样式 ======================================== */
.app-container {
  display: flex;
  justify-content: space-between; /* 子元素之间留有间隔 */
  margin: 0 auto;
  padding: 20px;
  font-family: Arial, sans-serif;
}

.content-left {
  flex: 1; 
  padding: 10px;
  width: 800px;
}

.content-right {
  flex: 1;
  padding: 10px;
  width: 400px;
}

h1 {
  text-align: center;
  color: #333;
}

h2 {
  color: #333;;
}

/* ======================================== Echart相关组件样式 ======================================== */
.status {
  text-align: center;
  padding: 10px;
  margin: 20px 0;
  background-color: #ffebee;
  color: #c62828;
  border-radius: 4px;
}

.status.connected {
  background-color: #e8f5e9;
  color: #2e7d32;
}

.angle-chart-container {
  width: 100%;
  height: 400px;
}
.chart {
  width: 100%;
  height: 100%;
}

/* ======================================== 控制面板样式 ======================================== */
.panel-button {
  display: inline-block;
  padding: 10px 20px;
  font-size: 16px;
  cursor: pointer;
  text-align: center;
  text-decoration: none;
  outline: none;
  color: #fff;
  background-color: skyblue;
  border: none;
  border-radius: 15px;
  box-shadow: 0 4px #9E9E9E;
  transition-duration: 0.4s; /* 鼠标悬停过渡效果 */
}

.panel-button:hover { /* 悬停时背景颜色变浅 */
  background-color: blue;
}

.panel-button:active { 
  background-color: skyblue;
  box-shadow: 0 2px #666;
  transform: translateY(2px);
}

.panel-wrapper {
  background-color: #f9f9f9; 
  border-radius: 8px; /* 圆角边框 */
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); /* 阴影效果 */
  padding: 20px;
  text-align: center;
}
</style>