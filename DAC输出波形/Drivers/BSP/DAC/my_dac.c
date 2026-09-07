#include "./BSP/DAC/my_dac.h"

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;

/* DAC1 init function */
void MX_DAC1_Init(void)
{
  __HAL_RCC_DAC12_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_DMA1_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0}; 
  DAC_ChannelConfTypeDef sConfig = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  hdac1.Instance = DAC1;
  HAL_DAC_Init(&hdac1);
 

  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2);
	
	
    hdma_dac1_ch2.Instance = DMA1_Stream1;
    hdma_dac1_ch2.Init.Request = DMA_REQUEST_DAC2;
    hdma_dac1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac1_ch2.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac1_ch2.Init.Mode = DMA_CIRCULAR;
    hdma_dac1_ch2.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_dac1_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_dac1_ch2);
    __HAL_LINKDMA(&hdac1,DMA_Handle2,hdma_dac1_ch2);	
		
		HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

