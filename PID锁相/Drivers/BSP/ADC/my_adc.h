#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void PeriphCommonClock_Config(void);

#endif

