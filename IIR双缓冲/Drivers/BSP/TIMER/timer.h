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


#define TIMX_INT_IRQn                  TIM6_DAC_IRQn
#define TIMX_INT_IRQHandler            TIM6_DAC_IRQHandler
#define TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)   /* TIM6 时钟使能 */

/******************************************************************************************/

extern TIM_HandleTypeDef htim7;  
void MX_TIM7_Init(void);

extern TIM_HandleTypeDef htim3;   //驱动ADC
void MX_TIM3_Init(void);

#endif




