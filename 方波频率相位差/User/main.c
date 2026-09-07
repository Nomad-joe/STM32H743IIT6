#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/SDRAM/sdram.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/timer.h"

#include "./CMSIS/DSP/Include/arm_math.h"

// ==================== 全局变量 ====================
volatile uint32_t ch3_capture = 0;
volatile uint32_t ch3_prev = 0;
volatile uint32_t ch4_capture = 0;
volatile uint32_t period = 0;
volatile int32_t  phase_diff = 0;
volatile uint8_t  new_data_ready = 0;
volatile uint8_t  first_capture = 1;

#define TIM5_CLK         240000000U
#define TIM5_MAX_VAL     0xFFFFFFFFU

// ==================== 计算相位差 ====================
int32_t calculate_phase_diff(uint32_t ch3_time, uint32_t ch4_time, uint32_t period)
{
    int32_t diff;
    
    if (ch4_time >= ch3_time) {
        // B信号在A信号之后触发（B滞后于A）
        diff = ch4_time - ch3_time;
    } else {
        // B信号在A信号之前触发（B超前于A）
        diff = (TIM5_MAX_VAL - ch3_time) + ch4_time + 1;
    }
    
    // diff 范围是 0 ~ period-1
    // 如果 diff > period/2，说明B实际上超前于A，转换为负值
    if (diff > (int32_t)(period / 2)) {
        diff = diff - period;  // 现在 diff 是负值，范围 -period/2 ~ -1
    }
    
    return diff;
}

// ==================== 主函数 ====================
int main(void)
{
    sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(192,5,2,4);
    delay_init(480);
    usart_init(115200);
    led_init();

    MX_TIM5_Init();
    
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);//PA2
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);//PA3
    HAL_TIM_Base_Start_IT(&htim5);

    while(1)
    {
        if (new_data_ready)
        {
            new_data_ready = 0;
            
            float freq = (period > 0) ? (float)TIM5_CLK / period : 0;
            
            // phase_diff 已经是正负值（计数单位）
            float phase = (period > 0) ? (float)phase_diff / period * 360.0f : 0;
            phase -=0.05;
					  phase+= 0.1;
            printf("Freq: %.2f Hz, Phase: %.2f deg\r\n", freq, phase);
        }
        delay_ms(50);
    }
}

// ===================== 捕获中断 =====================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t val;
    
    if (htim->Instance == TIM5)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            
            if (!first_capture) {
                if (val > ch3_prev) {
                    period = val - ch3_prev;
                } else {
                    period = (TIM5_MAX_VAL - ch3_prev) + val + 1;
                }
            }
            
            ch3_prev = val;
            ch3_capture = val;
            first_capture = 0;
            
            if (ch4_capture > 0 && period > 0) {
                phase_diff = calculate_phase_diff(ch3_capture, ch4_capture, period);
                new_data_ready = 1;
            }
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            ch4_capture = val;
            
            if (!first_capture && period > 0) {
                phase_diff = calculate_phase_diff(ch3_capture, ch4_capture, period);
                new_data_ready = 1;
            }
        }
    }
}