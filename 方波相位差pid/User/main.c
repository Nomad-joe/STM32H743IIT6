#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/SDRAM/sdram.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/timer.h"
#include "./BSP/AD9833/my_ad9833.h"

#include "./CMSIS/DSP/Include/arm_math.h"

volatile uint8_t  Tim5_cnt = 0;
volatile uint32_t cap_ref = 0;       // CH3 参考信号上升沿
volatile uint32_t cap_dds = 0;       // CH4 DDS信号上升沿
volatile uint32_t period_tick = 240000;
volatile int32_t  phase_delta = 0;
volatile float    phase_norm = 0.0f;
volatile float    phase_deg = 0.0f;  // 相位差（度）
volatile uint8_t  ch4_capture_ok = 0;

#define TIM5_MAX_VAL     4294967295U
#define TIM5_CLK         240000000U
#define TARGET_PHASE     90.0f



int main(void)
{
    sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(192,5,2,4);
    delay_init(480);
    usart_init(115200);  // 串口初始化 115200
    led_init();
    AD9833_Init();
    MX_TIM5_Init();
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim5);
  AD9833_WaveSeting1(1000,);
    while(1)
    {

			
			

            printf("相位差：%.2f °\r\n", phase_deg);

        delay_ms(50);
    }
}

// ===================== 捕获中断 =====================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t val;

    if(htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        period_tick = val - cap_ref;
        cap_ref = val;
        Tim5_cnt = 0;
    }

    if(htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
        cap_dds = val;

        phase_delta = (int32_t)(cap_dds - cap_ref);
        phase_norm = (float)phase_delta / period_tick;

        // 归一化到 [-0.5, 0.5]
        if(phase_norm > 0.5f)  phase_norm -= 1.0f;
        if(phase_norm < -0.5f) phase_norm += 1.0f;

        // 转换为 角度 (-180° ~ 180°)
        phase_deg = phase_norm * 360.0f;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM5)
        Tim5_cnt++;
}