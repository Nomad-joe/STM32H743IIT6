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
#define FFT_LENGTH		4096 		//FFT长度
extern uint16_t AD9226_Data[FFT_LENGTH+20];
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


void TIM3_Init(void);
void AD9226_Read_Data(uint8_t mode);
/******************************************************************************************/
/* 定时器 定义 */


#endif




