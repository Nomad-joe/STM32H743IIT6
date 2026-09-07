/**
 ****************************************************************************************************
 * @file        timer.c
 * @version     V2.0
 * @brief       TIM12: AD9226/DAC904 共享时钟 + DMA触发源
 ****************************************************************************************************
 * @attention   Waiken-Smart 科业致远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 * 架构设计 (参考网友 AD9220 DMA 采集方案):
 *   TIM12 CH2 (PH9): PWM 2MHz, 50%占空比 → AD9226 CLK + DAC904 CLK (共享时钟)
 *   TIM12 TRGO (Update): → DMAMUX ReqGen0 (DAC DMA Stream0, M→P→GPIOC)
 *                       → DMAMUX ReqGen1 (ADC DMA Stream1, GPIOI→M, 自动采集)
 *
 *   DMA 自动完成每个时钟周期的数据搬移:
 *     - ADC: GPIOI->IDR → AD9226_Data[] (P→M, 环形)
 *     - DAC: Filtered_Buffer[] → GPIOC->ODR (M→P, 环形)
 *   HT/TC 中断设置 Ping-Pong 标志, 主循环做 IFFT 滤波
 *
 * 时钟计算:
 *   定时器时钟 = 240MHz (APB1 Timer Clock)
 *   Prescaler = 1-1 = 0, Period = 120-1 = 119
 *   频率 = 240MHz / 120 = 2 MHz
 ****************************************************************************************************
 */

#include "./BSP/LED/led.h"
#include "./BSP/TIMER/timer.h"

uint16_t AD9226_Data[ADC_BUF_SIZE];
volatile uint8_t adc_half_flag = 0;
volatile uint8_t adc_full_flag = 0;

TIM_HandleTypeDef htim12;  // AD9226 + DAC904 共享时钟定时器

/**
 * @brief  TIM12 初始化 — 2MHz 共享时钟 + DMA触发源
 *         CH2 PWM输出 (2MHz, 50%占空比) → PH9 → AD9226 CLK + DAC904 CLK
 *         TRGO (Update) → DMAMUX ReqGen0/1 → 两个DMA同步工作
 *         频率 = 240MHz / 120 = 2 MHz
 */
void MX_TIM12_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim12.Instance = TIM12;
    htim12.Init.Prescaler = 1 - 1;          // 不分频, 240MHz
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim12.Init.Period = 120 - 1;            // 240MHz / 120 = 2MHz
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
    {
        Error_Handler();
    }
    // TRGO = Update event → 驱动两个 DMAMUX 请求生成器
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // CH2 PWM: 50% 占空比 → PH9 输出时钟
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 60;                    // 50% duty: 60/120
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim12);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM12)
    {
        __HAL_RCC_TIM12_CLK_ENABLE();
    }
}

/**
 * @brief  TIM12 MSP后初始化 — 配置PH9为TIM12_CH2 (AD9226 + DAC904 共享时钟)
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (timHandle->Instance == TIM12)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
        /** PH9 → TIM12_CH2 (AF2) — AD9226 + DAC904 共享时钟 */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    }
}
