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






TIM_HandleTypeDef htim12;  //驱动AD9226

 void MX_TIM12_Init(void)
  {
      __HAL_RCC_TIM12_CLK_ENABLE();
      __HAL_RCC_GPIOH_CLK_ENABLE();

      // ===== PH9 —AD9226 CLK 输出 =====
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;
      HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

      TIM_MasterConfigTypeDef sMasterConfig = {0};
      TIM_OC_InitTypeDef sConfigOC = {0};

      // ===== 时基: 240MHz / 120 = 2MHz =====
      htim12.Instance = TIM12;
      htim12.Init.Prescaler = 1-1;                     // 不分频, 240MHz
      htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim12.Init.Period = 30 - 1;                  // 8MHz
      htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
      HAL_TIM_Base_Init(&htim12);

      // ===== CH2: AD9226 CLK (输出到 PH9) =====
      sConfigOC.OCMode     = TIM_OCMODE_PWM1;
      sConfigOC.Pulse      = 15;                     // 50% 占空比
      sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;    // ★改为 HIGH
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
      HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2);

      // ===== CH1: 内部 DMA 触发基准 (不接引脚!) =====
      // OC1REF 在 CNT=30 产生下降沿 →DMAMUX FALLING →DMA 触发
      sConfigOC.Pulse      = 10;                     // ★触发点: HIGH电平中间 (125ns)
      HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);

      // ===== TRGO 改为 OC1REF =====
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;   // ★改为 OC1REF
      sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
      HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig);
  }
