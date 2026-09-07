#ifndef __DAC_H
#define __DAC_H

#include "./SYSTEM/sys/sys.h"

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch2;


void MX_DAC1_Init(void);



#endif
