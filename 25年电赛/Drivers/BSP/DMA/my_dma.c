#include "./BSP/DMA/my_dma.h"

DMA_HandleTypeDef hdma_dma_generator0;
/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_dma_generator0
  */
void MX_DMA_Init(void)
{

  /* Local variables */
  HAL_DMA_MuxRequestGeneratorConfigTypeDef pRequestGeneratorConfig = {0};

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_dma_generator0 on DMA1_Stream0 */
  hdma_dma_generator0.Instance = DMA1_Stream0;
  hdma_dma_generator0.Init.Request = DMA_REQUEST_GENERATOR0;
  hdma_dma_generator0.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dma_generator0.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dma_generator0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dma_generator0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dma_generator0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_dma_generator0.Init.Mode = DMA_CIRCULAR;
  hdma_dma_generator0.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_dma_generator0.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_dma_generator0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure the DMAMUX request generator for the selected DMA stream */
  pRequestGeneratorConfig.SignalID = HAL_DMAMUX1_REQ_GEN_TIM12_TRGO;
  pRequestGeneratorConfig.Polarity = HAL_DMAMUX_REQ_GEN_RISING;
  pRequestGeneratorConfig.RequestNumber = 1;
  if (HAL_DMAEx_ConfigMuxRequestGenerator(&hdma_dma_generator0, &pRequestGeneratorConfig) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_DMAEx_EnableMuxRequestGenerator(&hdma_dma_generator0) != HAL_OK)
{
    Error_Handler();
}

}
