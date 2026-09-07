/**
 ****************************************************************************************************
 * @file        timer.h
 * @version     V1.0
 * @brief       定时器驱动 — AD9226/DAC904 共享时钟 + DMA触发
 ****************************************************************************************************
 * @attention   Waiken-Smart 科业致远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 * TIM12 架构:
 *   CH2 (PH9): PWM 2MHz, 50%占空比 → AD9226 + DAC904 共享时钟
 *   TRGO (Update): → DMAMUX ReqGen0 (DAC输出DMA) + ReqGen1 (ADC采集DMA)
 *   两个DMA在每个时钟周期自动完成：GPIOI→ADC缓冲 和 滤波缓冲→GPIOC
 *
 ****************************************************************************************************
 */

#ifndef __TIM_H
#define __TIM_H

#include "./SYSTEM/sys/sys.h"

#define FFT_LENGTH      4096             // FFT长度
#define ADC_BUF_SIZE    (FFT_LENGTH * 2) // 双缓冲: 2 x 4096 = 8192

extern uint16_t AD9226_Data[ADC_BUF_SIZE];
extern TIM_HandleTypeDef htim12;
extern volatile uint8_t adc_half_flag;
extern volatile uint8_t adc_full_flag;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM12_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/******************************************************************************************/
/* 定时器 驱动 */

#endif
