<template>
  <div class="angle-chart-container">
    <v-chart class="chart" :option="chartOption" autoresize />
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
    const chartOption = ref({
      title: {
        text: '实时角度监控',
        left: 'center'
      },
      tooltip: {
        trigger: 'axis',
        formatter: '{b}<br/>角度: {c}°'
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
        name: '角度 (°)',
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
          name: '角度',
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

    const updateChart = (angle) => {
      const now = new Date();
      const time = `${now.getHours()}:${now.getMinutes()}:${now.getSeconds()}`;
      
      // 限制数据点数量
      if (chartOption.value.xAxis.data.length > 50) {
        chartOption.value.xAxis.data.shift();
        chartOption.value.series[0].data.shift();
      }
      
      chartOption.value.xAxis.data.push(time);
      chartOption.value.series[0].data.push(angle);
    };

    return {
      chartOption,
      updateChart
    };
  }
};
</script>

<style scoped>
.angle-chart-container {
  width: 100%;
  height: 400px;
}
.chart {
  width: 100%;
  height: 100%;
}
</style>