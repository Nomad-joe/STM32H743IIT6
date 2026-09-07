#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);


#endif

