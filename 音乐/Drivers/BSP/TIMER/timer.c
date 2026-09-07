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
#include "./SYSTEM/delay/delay.h"



TIM_HandleTypeDef htim2;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_TIM2_CLK_ENABLE();
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 480-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

}

/**
 * @brief  音乐 PWM 播放函数（H7IIT6 240MHz 专用）
 * @param  freq: 频率（音符）
 * @param  duty: 占空比 0~100（推荐 50）
 * @param  time_ms: 发声时长
 */
void Music_Tone(uint32_t freq, uint8_t duty, uint32_t time_ms)
{
    // 频率为0直接静音
    if(freq == 0)
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        delay_ms(time_ms);
        return;
    }

    // 限制占空比安全范围
    if(duty > 50) duty = 50;

    // ==================== H7 240MHz 自动计算 ====================
    const uint32_t TIM_CLK = 240000000;  // TIM2 时钟 240MHz
    uint32_t prescaler = 239;            // 分频 240 → 1MHz
    uint32_t period = 1000000 / freq - 1;
    uint32_t pulse = period * duty / 100;

    // 停止旧 PWM
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // 重新配置定时器
    htim2.Init.Prescaler = prescaler;
    htim2.Init.Period = period;
    HAL_TIM_PWM_Init(&htim2);

    // 重新配置 PWM 通道
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    // 启动发声
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // 播放指定时间（你自己的 delay 函数）
    delay_ms(time_ms);

    // 时间到 → 自动停止发声
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}




