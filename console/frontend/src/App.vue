<template>
  <div class="app-container">
    <div class="content-left">
      <h1>云台控制台</h1>
      <div class="status" :class="{ connected }">
        {{ connected ? '已连接' : '未连接' }}
      </div>
      <AngleChart ref="chart" />
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
import { ref, onMounted } from 'vue';
import AngleChart from './components/AngleChart.vue';

export default {
  components: { AngleChart },
  setup() {
    const connected = ref(false);
    const chart = ref(null);
    const SERVER_PORT = 8081;
    let socket = null;

    /**
     * WebSocket获取实时云台倾角
     */
    const connectWebSocket = () => {
      socket = new WebSocket(`ws://localhost:${SERVER_PORT}`);

      socket.onopen = () => {
        console.log('Connected to WebSocket server');
      };

      socket.onmessage = (event) => {
        const { type, data } = JSON.parse(event.data);
        
        if (type === 'angle') {
          const angleMatch = data.match(/Angle:([-\d.]+)/);
          if (angleMatch) {
            const angle = parseFloat(angleMatch[1]);
            chart.value.updateChart(angle);
          }
        } else if (type === 'status') {
          connected.value = data.isConnected;
        } else if (type === 'error') {
          console.error('Error:', data);
        }
      };

      socket.onclose = () => {
        connected.value = false;
        console.log('Disconnected from WebSocket server');
      };
    };

    /**
     * 发送旋转云台Post请求
     */
    const sendRotateRequest = () => {
      fetch(`http://localhost:${SERVER_PORT}/api/rotate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ action: 'rotate' })
      }).then(response => {
        if (response.ok) {
          console.log('Rotation request sent successfully');
        } else {
          return response.json().then(errData => {
            throw new Error(errData.message || 'Request failed');
          });
        }
      }).catch(error => {
        console.error('Error:', error);
        alert(`操作失败: ${error.message}`);
      });
    };

    onMounted(() => {
      connectWebSocket();
    });

    return {
      connected,
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