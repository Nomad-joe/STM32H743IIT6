/**
 ****************************************************************************************************
 * @file        my_dma.c
 * @version     V2.0
 * @brief       DMA 驱动程序 — AD9226 自动采集 + DAC904 自动输出
 ****************************************************************************************************
 * @attention   Waiken-Smart 科业致远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 * 架构 (参考网友 AD9220 DMA 采集方案):
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │ TIM12 CH2 (PH9) PWM 2MHz                                │
 * │   ├──→ AD9226 CLK  (硬件连线)                            │
 * │   └──→ DAC904 CLK  (硬件连线)                            │
 * │ TIM12 TRGO (Update)                                     │
 * │   ├──→ DMAMUX ReqGen0 → DMA1_Stream0 → M→P → GPIOC->ODR │
 * │   └──→ DMAMUX ReqGen1 → DMA1_Stream1 → P→M → GPIOI->IDR │
 * └─────────────────────────────────────────────────────────┘
 *
 * DMA1_Stream0: DAC904 数据输出 (M→P, Circular)
 *   由 DMAMUX Request Generator 0 触发, 源: TIM12_TRGO
 *   每次触发: Filtered_Buffer[n] → GPIOC->ODR (DAC904输出新数据)
 *
 * DMA1_Stream1: AD9226 数据采集 (P→M, Circular)
 *   由 DMAMUX Request Generator 1 触发, 源: TIM12_TRGO
 *   每次触发: GPIOI->IDR → AD9226_Data[n] (读12位ADC数据)
 *   HT/TC 中断 → adc_half_flag / adc_full_flag (Ping-Pong)
 ****************************************************************************************************
 */

#include "./BSP/DMA/my_dma.h"
#include "./BSP/TIMER/timer.h"

DMA_HandleTypeDef hdma_dma_generator0;    // DAC904 输出 DMA (DMA1_Stream0, ReqGen0)
DMA_HandleTypeDef hdma_adc_dma_generator1; // AD9226 采集 DMA (DMA1_Stream1, ReqGen1)

/* ---- AD9226 DMA 回调 (参考网友 AD9220_DMA_CpltCallback 方案) ---- */
static void AD9226_DMA_HalfCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma->Instance == DMA1_Stream1)
    {
        adc_half_flag = 1;   // 前半缓冲就绪 (0..4095)
    }
}

static void AD9226_DMA_CpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma->Instance == DMA1_Stream1)
    {
        adc_full_flag = 1;   // 后半缓冲就绪 (4096..8191)
    }
}

/**
 * @brief  DAC904 输出 DMA 初始化
 *         DMA1_Stream0, M→P, HalfWord, Circular
 *         DMAMUX ReqGen0 由 TIM12_TRGO 触发 (2MHz, 每周期推一个新数据到 DAC)
 */
void MX_DMA_Init(void)
{
    HAL_DMA_MuxRequestGeneratorConfigTypeDef pRequestGeneratorConfig = {0};

    __HAL_RCC_DMA1_CLK_ENABLE();

    // ========== DMA1_Stream0: DAC904 输出 (M→P) ==========
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

    // DMAMUX ReqGen0: TIM12_TRGO 触发, 每次触发搬1个 HalfWord
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

/**
 * @brief  AD9226 采集 DMA 初始化
 *         DMA1_Stream1, P→M, HalfWord, Circular
 *         DMAMUX ReqGen1 由 TIM12_TRGO 触发 (2MHz, 每个时钟周期自动读一次 ADC)
 *         HT/TC 中断用于 Ping-Pong 双缓冲处理
 *
 *         参考网友方案: 用 DMA 替代定时器 ISR 手动读 GPIO, 消除 CPU 开销,
 *         采样率可达 2Msps 以上
 */
void MX_ADC_DMA_Init(void)
{
    HAL_DMA_MuxRequestGeneratorConfigTypeDef pRequestGeneratorConfig = {0};

    __HAL_RCC_DMA1_CLK_ENABLE();

    // ========== DMA1_Stream1: AD9226 采集 (P→M) ==========
    hdma_adc_dma_generator1.Instance = DMA1_Stream1;
    hdma_adc_dma_generator1.Init.Request = DMA_REQUEST_GENERATOR1;
    hdma_adc_dma_generator1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc_dma_generator1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc_dma_generator1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc_dma_generator1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc_dma_generator1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc_dma_generator1.Init.Mode = DMA_CIRCULAR;
    hdma_adc_dma_generator1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc_dma_generator1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc_dma_generator1) != HAL_OK)
    {
        Error_Handler();
    }

    // DMAMUX ReqGen1: TIM12_TRGO 触发, 每次触发搬1个 HalfWord
    pRequestGeneratorConfig.SignalID = HAL_DMAMUX1_REQ_GEN_TIM12_TRGO;
    pRequestGeneratorConfig.Polarity = HAL_DMAMUX_REQ_GEN_RISING;
    pRequestGeneratorConfig.RequestNumber = 1;
    if (HAL_DMAEx_ConfigMuxRequestGenerator(&hdma_adc_dma_generator1, &pRequestGeneratorConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMAEx_EnableMuxRequestGenerator(&hdma_adc_dma_generator1) != HAL_OK)
    {
        Error_Handler();
    }

    // 注册 HT/TC 回调函数 (参考网友 HAL_DMA_RegisterCallback 方案)
    HAL_DMA_RegisterCallback(&hdma_adc_dma_generator1,
                             HAL_DMA_XFER_HALFCPLT_CB_ID,
                             AD9226_DMA_HalfCpltCallback);
    HAL_DMA_RegisterCallback(&hdma_adc_dma_generator1,
                             HAL_DMA_XFER_CPLT_CB_ID,
                             AD9226_DMA_CpltCallback);

    // 使能 HT 和 TC 中断用于 Ping-Pong 双缓冲
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    __HAL_DMA_ENABLE_IT(&hdma_adc_dma_generator1, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(&hdma_adc_dma_generator1, DMA_IT_TC);
}
