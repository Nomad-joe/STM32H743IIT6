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




TIM_HandleTypeDef htim8;

void TIM8_CH2_PI6_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* 1. 开时钟 */
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    /* 2. PI6 复用为 TIM8_CH2 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;        // 触发信号不需要上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /* 3. TIM8 基础配置：240MHz  */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 12-1;                  // 不分频
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 1000-1;                   // 自动重装载值
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim8);

    /* 4. PWM 通道配置 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;                     // 50% 占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);

    /* 注意：初始化时不启动 PWM！在 main 采集时才启动 */
    // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

