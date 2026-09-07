/**
 ****************************************************************************************************
 * @file        my_dma.h
 * @version     V2.0
 * @brief       DMA 驱动程序 — DAC904输出 + AD9226采集
 ****************************************************************************************************
 * @attention   Waiken-Smart 科业致远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 * DMA1_Stream0 (ReqGen0): Memory → GPIOC->ODR (DAC904数据输出, M→P, Circular)
 *   DMAMUX ReqGen0 触发源: TIM12_TRGO (2MHz)
 *
 * DMA1_Stream1 (ReqGen1): GPIOI->IDR → Memory (AD9226数据采集, P→M, Circular)
 *   DMAMUX ReqGen1 触发源: TIM12_TRGO (2MHz)
 *   HT/TC 中断设置 Ping-Pong 标志
 *
 * 两个 DMA 在同一 TIM12_TRGO 事件触发, 实现 ADC 采集与 DAC 输出同步
 ****************************************************************************************************
 */

#ifndef __DMA_H
#define __DMA_H

#include "./SYSTEM/sys/sys.h"

extern DMA_HandleTypeDef hdma_dma_generator0;   // DAC904 输出 DMA (Stream0)
extern DMA_HandleTypeDef hdma_adc_dma_generator1; // AD9226 采集 DMA (Stream1)

void MX_DMA_Init(void);       // DAC904 输出 DMA 初始化
void MX_ADC_DMA_Init(void);   // AD9226 采集 DMA 初始化 (DMAMUX ReqGen1)

#endif
