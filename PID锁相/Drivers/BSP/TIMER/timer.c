/**
 ****************************************************************************************************
 * @file        timer.c
 * @version     V1.0
 * @brief       定时器中断 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 ****************************************************************************************************
 */

#include "./BSP/LED/led.h"
#include "./BSP/TIMER/timer.h"


TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;       /* 定时器TIMX句柄 */

void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1172-1; //采样率 ：204,778  1024 点分辨率 ：199.978 近似200
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}



extern uint8_t g_timeout;              /* 在main.c里面定义 */

void timx_int_init(uint32_t arr, uint16_t psc)
{  
   	__HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance = TIM6;                            /* 定时器TIMX */
    htim6.Init.Prescaler = psc;                       /* 设置预分频系数 */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;      /* 递增计数模式 */
    htim6.Init.Period = arr;                          /* 设置自动重载值 */
    HAL_TIM_Base_Init(&htim6);
                                   /* 使能TIMX时钟 */
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 3);            /* 设置中断优先级，抢占优先级1，子优先级3 */
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);                    /* 使能TIMX中断 */
    HAL_TIM_Base_Start_IT(&htim6);                    /* 使能定时器TIMX和定时器TIMX更新中断 */
}
	
void TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);                       /* 调用HAL库定时器中断公共处理函数 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        g_timeout++;               
    }
}

