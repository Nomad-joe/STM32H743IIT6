/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V2.0
 * @brief       AD9226 DMA自动采集 + IFFT实时滤波 + DAC904 DMA自动输出
 ****************************************************************************************************
 * @attention   Waiken-Smart 科业致远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 * 架构 (参考网友 AD9220 DMA 采集方案):
 *   TIM12 CH2 (PH9): PWM 2MHz → AD9226 CLK + DAC904 CLK (共享时钟)
 *   TIM12 TRGO → DMAMUX ReqGen0 → DMA1_Stream0 → M→P → DAC904(GPIOC)
 *   TIM12 TRGO → DMAMUX ReqGen1 → DMA1_Stream1 → P→M → AD9226(GPIOI)
 *
 *   DMA 自动采集: 每个时钟周期 DMA 自动从 GPIOI->IDR 读取12位ADC数据
 *                写入环形缓冲 AD9226_Data[8192], 无需 CPU 参与
 *   Ping-Pong: HT中断(前半就绪) / TC中断(后半就绪) → IFFT滤波处理
 *   DMA 自动输出: 每个时钟周期 DMA 自动推送滤波数据到 GPIOC->ODR
 *
 * 采样率: 2 Msps  |  FFT长度: 4096  |  双缓冲半区: 4096点(2.048ms)
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/SDRAM/sdram.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/timer.h"
#include "./BSP/ADC/my_adc.h"
#include "./BSP/FFT/my_fft.h"
#include "./CMSIS/DSP/Include/arm_math.h"
#include "./BSP/DMA/my_dma.h"

#include "math.h"

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;

#define FFT_LEN  4096
#define CUTOFF_FREQ  20000.0f           // 低通截止频率 20 kHz
#define SAMPLE_FREQ  2000000.0f         // 采样率 2 MHz
#define CUTOFF_BIN   ((uint32_t)(CUTOFF_FREQ * FFT_LEN / SAMPLE_FREQ))  // ≈ 41

/* ---- 全局缓冲 ---- */
uint16_t Filtered_Buffer[ADC_BUF_SIZE]; // 滤波输出, DMA推送到DAC904

/* ---- FFT工作缓冲 (全局, 避免栈溢出) ---- */
static float FFT_input[2 * FFT_LEN];    // 复数FFT输入/输出

/* ---- DAC/ADC DMA启动标志 ---- */
static uint8_t dac_dma_started = 0;
static uint8_t adc_dma_started = 0;


/**
 * @brief  IFFT频域滤波处理
 * @param  src : 输入ADC数据 (4096点 uint16_t)
 * @param  dst : 输出DAC数据 (4096点 uint16_t, 14-bit范围)
 * @note   流程: ADC码→归一化→加窗→FFT→频率掩码→IFFT→缩放→DAC码
 */
static void IFFT_Filter_Process(uint16_t *src, uint16_t *dst)
{
    uint32_t i;

    /* === 1. ADC码 → 归一化浮点 ±1, 并加 Hanning 窗 === */
    for (i = 0; i < FFT_LEN; i++)
    {
        /* 提取12位有效数据, 映射到 -1.0 ~ +1.0 */
        float x = ((float)(src[i] & 0x0FFF) / 4095.0f) * 2.0f - 1.0f;

        /* Hanning窗: w[n] = 0.5 * (1 - cos(2πn/(N-1))) */
        float win = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_LEN - 1)));

        FFT_input[2 * i]     = x * win;  /* 实部 */
        FFT_input[2 * i + 1] = 0.0f;     /* 虚部 */
    }

    /* === 2. 4096点复数 FFT (正向) === */
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);

    /* === 3. 频域低通掩码: 保留 bin [0..CUTOFF_BIN], 置零其余 === */
    for (i = CUTOFF_BIN + 1; i < FFT_LEN / 2; i++)
    {
        FFT_input[2 * i]     = 0.0f;     /* 正频率置零 */
        FFT_input[2 * i + 1] = 0.0f;

        /* 对称处理负频率部分 */
        FFT_input[2 * (FFT_LEN - i)]     = 0.0f;
        FFT_input[2 * (FFT_LEN - i) + 1] = 0.0f;
    }

    /* === 4. 4096点复数 IFFT (逆向) === */
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 1, 1);

    /* === 5. 缩放 + 转换为 DAC 码 (14-bit: 0 ~ 16383) === */
    /*     CMSIS-DSP IFFT 不做 1/N 缩放, 需手动除以 FFT_LEN */
    /*     信号范围: -1 ~ +1 → DAC码: 0 ~ 16383 */
    for (i = 0; i < FFT_LEN; i++)
    {
        float y = FFT_input[2 * i] / (float)FFT_LEN;  /* IFFT缩放 */

        /* ±1 → 0..16383, 钳位到有效范围 */
        float dac_val = (y + 1.0f) * 0.5f * 16383.0f;
        if (dac_val > 16383.0f) dac_val = 16383.0f;
        if (dac_val < 0.0f)     dac_val = 0.0f;

        dst[i] = (uint16_t)(dac_val + 0.5f);
    }
}


/**
 * @brief  main 主函数
 */
int main(void)
{
    sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480MHz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */

    printf("=== AD9226 DMA + IFFT Real-Time Filter ===\r\n");
    printf("Sample Rate: %.0f Hz, FFT: %d pts, Cutoff: %.0f Hz\r\n",
           SAMPLE_FREQ, FFT_LEN, CUTOFF_FREQ);
    printf("AD9226+DAC904 share CLK: TIM12 CH2 (PH9)\r\n");

    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护重要存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Step 1: GPIO初始化 (AD9226数据口 GPIOI + DAC904数据口 GPIOC) */
    MX_GPIO_Init();

    /* Step 2: TIM12 共享时钟 + DMA触发源
     *         CH2 (PH9) PWM 2MHz → AD9226 CLK + DAC904 CLK
     *         TRGO Update → DMAMUX ReqGen0/1 → DAC DMA + ADC DMA
     */
    MX_TIM12_Init();

    /* Step 3: DAC904 输出 DMA (DMA1_Stream0, ReqGen0, M→P, TIM12_TRGO触发) */
    MX_DMA_Init();

    /* Step 4: AD9226 采集 DMA (DMA1_Stream1, ReqGen1, P→M, TIM12_TRGO触发)
     *         DMA 自动从 GPIOI->IDR 读取 ADC 数据到 AD9226_Data[]
     *         HT/TC 中断设置 Ping-Pong 标志, CPU 零开销采集
     */
    MX_ADC_DMA_Init();

    /* Step 5: 启动 AD9226 采集 DMA (环形模式, 持续采集) */
    HAL_DMA_Start_IT(&hdma_adc_dma_generator1,
                     (uint32_t)&GPIOI->IDR,
                     (uint32_t)AD9226_Data,
                     ADC_BUF_SIZE);
    adc_dma_started = 1;
    printf("ADC DMA Started (DMA1_Stream1, auto from GPIOI).\r\n");

    /* Step 6: 启动 TIM12 PWM → 时钟开始输出, 两个 DMA 同步工作 */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    printf("TIM12 CH2 (PH9) Clock Started. All DMAs running.\r\n");

    /* ---- 主循环: Ping-Pong 双缓冲处理 ---- */
    while (1)
    {
        /* 前半缓冲就绪: AD9226_Data[0 .. FFT_LEN-1] (DMA HT中断触发) */
        if (adc_half_flag == 1)
        {
            adc_half_flag = 0;
        for(uint16_t i=0;i<FFT_LEN;i++)
					printf("%d\r\n",AD9226_Data[i]);
#if 0
            /* IFFT滤波: 前半输入 → 前半输出 */
            IFFT_Filter_Process(&AD9226_Data[0], &Filtered_Buffer[0]);

            /* 首次处理完成后启动DAC DMA (仅一次) */
            if (dac_dma_started == 0)
            {
                dac_dma_started = 1;
                HAL_DMA_Start(&hdma_dma_generator0,
                              (uint32_t)Filtered_Buffer,
                              (uint32_t)&GPIOC->ODR,
                              ADC_BUF_SIZE);
                printf("DAC DMA Started.\r\n");
            }
#endif					
        }

        /* 后半缓冲就绪: AD9226_Data[FFT_LEN .. 2*FFT_LEN-1] (DMA TC中断触发) */
        if (adc_full_flag == 1)
        {
        adc_full_flag = 0;
        for(uint16_t i=FFT_LEN;i<2*FFT_LEN;i++)
					printf("%d\r\n",AD9226_Data[i]);
            /* IFFT滤波: 后半输入 → 后半输出 */
   //         IFFT_Filter_Process(&AD9226_Data[FFT_LEN], &Filtered_Buffer[FFT_LEN]);
        }
    }
}
