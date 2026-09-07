/**
 ****************************************************************************************************
 * @file        timer.c
 * @version     V1.0
 * @brief       ��ʱ���ж� ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    STM32H743IIT6Сϵͳ��
 *
 ****************************************************************************************************
 */

#include "./BSP/LED/led.h"
#include "./BSP/TIMER/timer.h"






TIM_HandleTypeDef htim12;  //����AD9226

 void MX_TIM12_Init(void)
  {
      __HAL_RCC_TIM12_CLK_ENABLE();
      __HAL_RCC_GPIOH_CLK_ENABLE();

      // ===== PH9 ��AD9226 CLK ��� =====
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;
      HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

      TIM_MasterConfigTypeDef sMasterConfig = {0};
      TIM_OC_InitTypeDef sConfigOC = {0};

      // ===== ʱ��: 240MHz / 120 = 2MHz =====
      htim12.Instance = TIM12;
      htim12.Init.Prescaler = 1-1;                     // ����Ƶ, 240MHz
      htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim12.Init.Period = 15 - 1;                  // 16MHz
      htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
      HAL_TIM_Base_Init(&htim12);

      // ===== CH2: AD9226 CLK (����� PH9) =====
      sConfigOC.OCMode     = TIM_OCMODE_PWM1;
      sConfigOC.Pulse      = 7;                      // ~47% ռ�ձ�
      sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;    // ���Ϊ HIGH
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
      HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2);

      // ===== CH1: �ڲ� DMA ������׼ (��������!) =====
      // OC1REF �� CNT=30 �����½��� ��DMAMUX FALLING ��DMA ����
      sConfigOC.Pulse      = 5;                      // �ﴥ����: HIGH��ƽ�м� (~104ns)
      HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);

      // ===== TRGO ��Ϊ OC1REF =====
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;   // ���Ϊ OC1REF
      sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
      HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig);
  }
