#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void MX_ADC1_Init(void);

#endif

