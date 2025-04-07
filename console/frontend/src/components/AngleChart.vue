<template>
  <div class="angle-chart-container">
    <div class="chart-group">
      <div class="chart-wrapper">
        <v-chart class="chart" :option="speedChartOption" autoresize />
      </div>
      <div class="chart-wrapper">
        <v-chart class="chart" :option="accelerationChartOption" autoresize />
      </div>
    </div>
  </div>
</template>

<script>
import { ref, onMounted } from 'vue';
import { use } from 'echarts/core';
import { CanvasRenderer } from 'echarts/renderers';
import { LineChart } from 'echarts/charts';
import {
  GridComponent,
  TooltipComponent,
  TitleComponent
} from 'echarts/components';
import VChart from 'vue-echarts';

use([
  CanvasRenderer,
  LineChart,
  GridComponent,
  TooltipComponent,
  TitleComponent
]);

export default {
  components: { VChart },
  props: {
    angleData: {
      type: Array,
      default: () => []
    }
  },
  
  setup(props) {
    // Speed Chart Configuration
    const speedChartOption = ref({
      title: {
        text: '实时速度监控',
        left: 'center'
      },
      tooltip: {
        trigger: 'axis',
        formatter: '{b}<br/>速度: {c}°'
      },
      xAxis: {
        type: 'category',
        data: [],
        axisLabel: {
          show: false
        }
      },
      yAxis: {
        type: 'value',
        name: '速度',
        min: -30,
        max: 30,
        axisLine: {
          lineStyle: {
            color: '#999'
          }
        },
        splitLine: {
          lineStyle: {
            type: 'dashed'
          }
        }
      },
      series: [
        {
          name: '速度',
          type: 'line',
          smooth: true,
          data: [],
          lineStyle: {
            width: 3,
            color: '#5470C6'
          },
          itemStyle: {
            color: '#5470C6'
          },
          markLine: {
            silent: true,
            data: [
              {
                yAxis: 0,
                lineStyle: {
                  color: '#FF5722',
                  type: 'dashed'
                }
              }
            ]
          }
        }
      ],
      animationDuration: 300
    });

    // Acceleration Chart Configuration
    const accelerationChartOption = ref({
      title: {
        text: '实时加速度监控',
        left: 'center'
      },
      tooltip: {
        trigger: 'axis',
        formatter: '{b}<br/>加速度: {c} m/s²'
      },
      xAxis: {
        type: 'category',
        data: [],
        axisLabel: {
          show: false
        }
      },
      yAxis: {
        type: 'value',
        name: '加速度',
        min: -5,
        max: 5,
        axisLine: {
          lineStyle: {
            color: '#999'
          }
        },
        splitLine: {
          lineStyle: {
            type: 'dashed'
          }
        }
      },
      series: [
        {
          name: '加速度',
          type: 'line',
          smooth: true,
          data: [],
          lineStyle: {
            width: 3,
            color: '#91CC75'
          },
          itemStyle: {
            color: '#91CC75'
          },
          markLine: {
            silent: true,
            data: [
              {
                yAxis: 0,
                lineStyle: {
                  color: '#FF5722',
                  type: 'dashed'
                }
              }
            ]
          }
        }
      ],
      animationDuration: 300
    });

    const updateChart = (speed, acceleration) => {
      const now = new Date();
      const time = `${now.getHours()}:${now.getMinutes()}:${now.getSeconds()}`;
      
      // Update speed chart
      if (speedChartOption.value.xAxis.data.length > 50) {
        speedChartOption.value.xAxis.data.shift();
        speedChartOption.value.series[0].data.shift();
      }
      speedChartOption.value.xAxis.data.push(time);
      speedChartOption.value.series[0].data.push(speed);
      
      // Update acceleration chart
      if (accelerationChartOption.value.xAxis.data.length > 50) {
        accelerationChartOption.value.xAxis.data.shift();
        accelerationChartOption.value.series[0].data.shift();
      }
      accelerationChartOption.value.xAxis.data.push(time);
      accelerationChartOption.value.series[0].data.push(acceleration);
    };

    return {
      speedChartOption,
      accelerationChartOption,
      updateChart
    };
  }
};
</script>

<style scoped>
.angle-chart-container {
  width: 100%;
  height: 600px; /* Increased height to accommodate two charts */
}

.chart-group {
  display: flex;
  flex-direction: column;
  gap: 20px;
  height: 100%;
}

.chart-wrapper {
  flex: 1;
  height: 50%;
}

.chart {
  width: 100%;
  height: 100%;
}
</style>