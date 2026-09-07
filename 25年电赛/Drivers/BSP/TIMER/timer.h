/**
 ****************************************************************************************************
 * @file        timer.h
 * @version     V1.0
 * @brief       定时器中断 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 ****************************************************************************************************
 */

#ifndef __TIM_H
#define __TIM_H

#include "./SYSTEM/sys/sys.h"

extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;   //驱动adc3
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* TIM4 init function */
void MX_TIM4_Init(void);
void MX_TIM12_Init(void);
void MX_TIM3_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/******************************************************************************************/
/* 定时器 定义 */


#endif




