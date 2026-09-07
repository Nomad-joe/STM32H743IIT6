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


TIM_HandleTypeDef g_timx_handle;       /* 定时器TIMX句柄 */

extern uint8_t g_timeout;              /* 在main.c里面定义 */

TIM_HandleTypeDef htim3;

/**
 * @brief       定时器TIMX定时中断初始化函数
 * @note
 *              定时器的时钟来自APB1或APB2,如果D2PPREx对应于1或2分频，
 *              定时器的时钟为rcc_hclk1, 而rcc_hclk1为240M, 所以定时器时钟 = 240Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void timx_int_init(uint32_t arr, uint16_t psc)
{
    g_timx_handle.Instance = TIMX_INT;                        /* 定时器TIMX */
    g_timx_handle.Init.Prescaler = psc;                       /* 设置预分频系数 */
    g_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;      /* 递增计数模式 */
    g_timx_handle.Init.Period = arr;                          /* 设置自动重载值 */
    HAL_TIM_Base_Init(&g_timx_handle);
    
                                                              /* 使能定时器TIMX和定时器TIMX更新中断 */
}

/**
 * @brief       定时器底层驱动，开启时钟，设置中断优先级
 * @note        此函数会被HAL_TIM_Base_Init()函数调用
 * @param       htim  : 定时器句柄
 * @retval      无
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIMX_INT)
    {
        TIMX_INT_CLK_ENABLE();                                /* 使能TIMX时钟 */
        HAL_NVIC_SetPriority(TIMX_INT_IRQn, 1, 3);            /* 设置中断优先级，抢占优先级1，子优先级3 */
        HAL_NVIC_EnableIRQ(TIMX_INT_IRQn);                    /* 使能TIMX中断 */
    }
}

/**
 * @brief       定时器TIMX中断服务函数
 * @param       无
 * @retval      无
 */
void TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_handle);                       /* 调用HAL库定时器中断公共处理函数 */
}

/**
 * @brief       定时器更新中断回调函数
 * @param       htim  : 定时器句柄
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIMX_INT)
    {
        g_timeout++;               /* 计时增加 */
    }
}



/* TIM3 init function */
void MX_TIM3_Init(void)
{

  __HAL_RCC_TIM3_CLK_ENABLE();

   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 30-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}





