/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V3.0
 * @brief       AD9226 8MHz PWM DMA + 16384点自研FFT + 电赛双窗FFT/谐波分析
 *
 *  硬件连接:
 *    PH9  (TIM12_CH2) -> AD9226 CLK  (8MHz PWM)
 *    PI0~PI11         -> AD9226 D0~D11 (12-bit 并行数据)
 *    PB10/PB11        -> USART3 淘晶驰屏幕
 *
 *  架构:
 *    TIM12_CH2 8MHz PWM -> AD9226 CLK
 *    TIM12_CH1 OC1REF -> TRGO -> DMAMUX Generator(FALLING) -> DMA1_Stream0
 *    DMA Normal 模式: 填满后自动停止 -> 主循环检测 -> 停 PWM -> FFT -> 重开
 *
 *  FFT: 自研16384点基-2 DIT (参考工程), 替换 CMSIS-DSP arm_cfft_f32
 *
 *  算法 (保留电赛逻辑):
 *    Pass 1: Hann窗 -> cfft -> 频率估计 (三点能量插值)
 *    Pass 2: Flat-top窗 -> cfft -> 幅度估计 + 谐波分析
 *    Vpp/Vrms 8次滑动平均 -> 串口屏显示
 *    波形/频谱 -> 串口屏绘图 (按键/命令触发)
 *
 *  调试: 串口 USART3 PB10/PB11 连接到淘晶驰屏幕
 ****************************************************************************************************
 * @attention   Waiken-Smart
 * 实现平台:    STM32H743IIT6
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

/* ============================================================================
 *  一、可调参数
 * ============================================================================ */

#define Print_screen        1                       /* 1=使能串口屏输出 */

#define FFT_LEN             16384                   /* FFT 点数 (2的幂)              */
#define SETTLING_SKIP       10                      /* 丢弃前 N 个采样点(ADC建立时间) */
#define SAMPLE_RATE_HZ      8000000.0f              /* TIM12 8MHz PWM 采样率         */

/* 抗干扰: 900KHz 以上频谱置零 (900000 / (8M/16384) ≈ 1844 bin) */
#define INTERFERENCE_CUTOFF_BIN  1844

/* ============================================================================
 *  二、全局缓冲区
 * ============================================================================ */

/* DMA 采集缓冲区 (含前导裕量) */
uint16_t DMA_Buffer[FFT_LEN + SETTLING_SKIP] = {0};

/* 电压转换缓冲区 */
float AD9226_Voltage[FFT_LEN + SETTLING_SKIP];

/* FFT: 自研复数结构体数组 (16384点) */
struct compx s1[MAX_FFT_N];

/* DMA 一帧完成标志: ISR 中写 1, 主循环检测后清 0 */
volatile uint8_t g_dma_done = 0;

/* ============================================================================
 *  三、电赛算法变量 (保留原命名)
 * ============================================================================ */

uint32_t Index = 0;             /* 幅度谱峰值索引 */
uint32_t Index2 = 0;
uint32_t Index3 = 0;
float Finally_Index;            /* 插值后的精确频率索引 */
float Finally_Freq;             /* 估计频率 (KHz)      */
float Finally_Amp;              /* 估计幅度 (mVpp)     */
float FFT_Amp;                  /* 峰值幅度 (raw)      */

volatile uint8_t waveform_request = 0;  /* 0=none, 1=1周期, 3=3周期 */

/* FFT 结果: 最多3个峰值 (基波 + 2谐波), 按频率升序 */
float all_freq[3], all_amp[3];
uint8_t all_cnt;

/* 8次滑动平均累加器 (Vpp/Vrms) */
float vpp_acc = 0.0f, vrms_acc = 0.0f;
uint8_t meas_cnt = 0;

/* 保存的幅度谱 (Flat-top FFT, 谐波掩蔽前) */
float spectrum_mag[FFT_LEN / 2];

/* ============================================================================
 *  四、函数声明
 * ============================================================================ */

void AD9226_ConvertToVoltage(uint16_t *adc_buf, float *vol_buf, uint32_t len);
void Send_Waveform(uint8_t mode);
void Send_Spectrum(void);
void RX_Change(void);
void Rx_Delete(void);

/* ============================================================================
 *  五、DMA 完成中断回调
 * ============================================================================ */

static void AD9226_DMA_CpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma->Instance == hdma_dma_generator0.Instance)
    {
        g_dma_done = 1;
    }
}

void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    AD9226_DMA_CpltCallback(hdma);
}

/* ============================================================================
 *  六、主函数
 * ============================================================================ */

int main(void)
{
    /* ---- 6.1 系统初始化 ---- */
    sys_cache_enable();                         /* 使能 L1-Cache              */
    HAL_Init();
    sys_stm32_clock_init(192, 5, 2, 4);        /* HSE 25M -> CPU 480MHz      */
    delay_init(480);
    led_init();
    mpu_memory_protection();
    sdram_init();
    key_init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* ---- 6.2 外设初始化 ---- */
    MX_GPIO_Init();                             /* PI0~PI11 配置为输入       */
    MX_USART3_UART_Init(115200);                /* PB10/PB11 -> 淘晶驰屏幕   */
    MX_TIM12_Init();                            /* PH9 PWM 8MHz + DMA触发    */
    MX_DMA_Init();                              /* DMA1_Stream0 + DMAMUX Gen */

    /* 注册 DMA 完成回调 */
    HAL_DMA_RegisterCallback(&hdma_dma_generator0,
                             HAL_DMA_XFER_CPLT_CB_ID,
                             AD9226_DMA_CpltCallback);
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /* ---- 6.3 主循环 ---- */
    while (1)
    {
        /* ----- 6.3.1 启动 PWM + DMA 采集 ----- */
        g_dma_done = 0;

        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);      /* PH9: AD9226 CLK      */
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);      /* DMA 触发源           */
        HAL_DMA_Start_IT(&hdma_dma_generator0,
                         (uint32_t)&GPIOI->IDR,          /* 源: PI口输入寄存器   */
                         (uint32_t)DMA_Buffer,            /* 目标: 缓冲区         */
                         FFT_LEN + SETTLING_SKIP);        /* 长度: 16384+10       */

        /* ----- 6.3.2 等待 DMA 完成 (带超时) ----- */
        uint32_t timeout = 0;
        while (!g_dma_done && timeout < 5000)
        {
            delay_ms(1);
            timeout++;
        }

        /* ----- 6.3.3 停采集 ----- */
        HAL_DMA_Abort(&hdma_dma_generator0);
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);

        if (!g_dma_done) continue;                      /* 超时重试 */

        /* ----- 6.3.4 原始数据 -> 电压 ----- */
        AD9226_ConvertToVoltage(DMA_Buffer, AD9226_Voltage, FFT_LEN + SETTLING_SKIP);

        /* ================================================================
         *  PASS 1: Hann 窗 -> cfft -> 频率估计
         *  Hann窗: 主瓣窄, 频率分辨率好
         * ================================================================ */
        for (uint16_t i = 0; i < FFT_LEN; i++)
        {
            float w_hann = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_LEN - 1)));
            s1[i].real = AD9226_Voltage[i + SETTLING_SKIP] * w_hann;
            s1[i].imag = 0.0f;
        }

        cfft(s1, FFT_LEN);

        /* 计算幅度 -> 存入 s1[].real */
        for (uint16_t i = 0; i < FFT_LEN; i++)
        {
            arm_sqrt_f32(s1[i].real * s1[i].real + s1[i].imag * s1[i].imag,
                         &s1[i].real);
        }

        /* 抑制直流 bin */
        s1[0].real = 0.0f;
        s1[1].real = 0.0f;
        s1[2].real = 0.0f;

        /* 抗干扰: 900KHz 以上频谱直接置零 */
        for (uint16_t i = INTERFERENCE_CUTOFF_BIN; i < FFT_LEN / 2; i++)
            s1[i].real = 0.0f;

        /* 手动找峰值 (替代 arm_max_f32) */
        FFT_Amp = 0.0f;
        Index = 0;
        for (uint16_t i = 0; i < FFT_LEN / 2; i++)
        {
            if (s1[i].real > FFT_Amp)
            {
                FFT_Amp = s1[i].real;
                Index = i;
            }
        }

        if (Index < 3)  Index = 3;
        if (Index > FFT_LEN / 2 - 2) Index = FFT_LEN / 2 - 2;

        /* 三点能量加权插值 (频率) */
        {
            float mL = s1[Index - 1].real, mC = s1[Index].real, mR = s1[Index + 1].real;
            float eL = mL * mL, eC = mC * mC, eR = mR * mR;
            float eSum = eL + eC + eR;
            Finally_Index = ((Index - 1) * eL + Index * eC + (Index + 1) * eR) / eSum;
            Finally_Freq = Finally_Index * 488.28125f / 1000.0f;   /* bin=8M/16384 Hz -> KHz */
        }

        /* ================================================================
         *  PASS 2: Flat-top 窗 -> cfft -> 幅度估计 + 谐波分析
         *  Flat-top: scalloping loss < 0.005 dB, 幅度精度极高
         * ================================================================ */
        {
            /* ISO 18431-1 5-term flat-top 系数 */
            const float FT_a0 = 0.21557895f;
            const float FT_a1 = 0.41663158f;
            const float FT_a2 = 0.277263158f;
            const float FT_a3 = 0.083578947f;
            const float FT_a4 = 0.006947368f;

            for (uint16_t i = 0; i < FFT_LEN; i++)
            {
                float theta = 2.0f * PI * i / (FFT_LEN - 1);
                float w_ft = FT_a0
                           - FT_a1 * arm_cos_f32(theta)
                           + FT_a2 * arm_cos_f32(2.0f * theta)
                           - FT_a3 * arm_cos_f32(3.0f * theta)
                           + FT_a4 * arm_cos_f32(4.0f * theta);
                s1[i].real = AD9226_Voltage[i + SETTLING_SKIP] * w_ft;
                s1[i].imag = 0.0f;
            }
        }

        cfft(s1, FFT_LEN);

        /* 手动计算幅度 (保留复数用于可能的相位分析) */
        for (uint16_t i = 0; i < FFT_LEN; i++)
        {
            float re = s1[i].real;
            float im = s1[i].imag;
            arm_sqrt_f32(re * re + im * im, &s1[i].real);
        }

        /* 抑制直流 & 近直流 bin */
        s1[0].real = 0.0f;
        s1[1].real = 0.0f;
        s1[2].real = 0.0f;
        s1[3].real = 0.0f;

        /* 抗干扰: 900KHz 以上频谱直接置零 */
        for (uint16_t i = INTERFERENCE_CUTOFF_BIN; i < FFT_LEN / 2; i++)
            s1[i].real = 0.0f;

        /* 保存干净频谱 (谐波掩蔽前) */
        for (uint16_t si = 0; si < FFT_LEN / 2; si++)
            spectrum_mag[si] = s1[si].real;

        /* 找 Flat-top 峰值 */
        FFT_Amp = 0.0f;
        Index = 0;
        for (uint16_t i = 0; i < FFT_LEN / 2; i++)
        {
            if (s1[i].real > FFT_Amp)
            {
                FFT_Amp = s1[i].real;
                Index = i;
            }
        }

        /* Flat-top 幅度校准: FFT_Amp * (2000mV / (N * coherent_gain)) */
        Finally_Amp = FFT_Amp * 2000.0f / ((float)FFT_LEN * 0.21557895f);

        /* ================================================================
         *  谐波分析: 基波 + 最多2个谐波 (>17mV)
         * ================================================================ */
        {
            const float BIN_WIDTH = 488.28125f;         /* Fs/N = 8M/16384 Hz */
            const float FT_NORM   = 2000.0f / ((float)FFT_LEN * 0.21557895f);
            const float MIN_AMP_MV = 17.0f;
            const uint32_t MASK_HALF = 15;              /* 掩蔽 +/-15 bin */

            uint32_t fund_bin = Index;
            float fund_amp_mv  = Finally_Amp;
            float fund_freq_khz = Finally_Freq;

            /* 掩蔽直流 + 基波区域 */
            for (uint32_t b = 0; b <= 2; b++) s1[b].real = 0.0f;
            int32_t m0 = (int32_t)fund_bin - (int32_t)MASK_HALF;
            int32_t m1 = (int32_t)fund_bin + (int32_t)MASK_HALF;
            if (m0 < 3) m0 = 3;
            if (m1 >= FFT_LEN / 2) m1 = FFT_LEN / 2 - 1;
            for (int32_t b = m0; b <= m1; b++) s1[b].real = 0.0f;

            /* 找谐波峰值 (最多2个) */
            float harm_amp[2] = {0.0f, 0.0f};
            float harm_freq[2] = {0.0f, 0.0f};

            for (uint8_t n = 0; n < 2; n++)
            {
                float pk = 0.0f;
                uint32_t bi = 0;

                for (uint32_t i = 0; i < FFT_LEN / 2; i++)
                {
                    if (s1[i].real > pk)
                    {
                        pk = s1[i].real;
                        bi = i;
                    }
                }

                if (bi < 3 || bi >= FFT_LEN / 2 - 2) break;

                float amp_mv = pk * FT_NORM;
                if (amp_mv < MIN_AMP_MV) break;

                harm_amp[n] = amp_mv;

                /* 3点插值求谐波频率 */
                float mL = s1[bi - 1].real, mC = s1[bi].real, mR = s1[bi + 1].real;
                float eL = mL * mL, eC = mC * mC, eR = mR * mR;
                float eSum = eL + eC + eR;
                float interp_idx = ((bi - 1) * eL + bi * eC + (bi + 1) * eR) / eSum;
                harm_freq[n] = interp_idx * BIN_WIDTH / 1000.0f;   /* -> KHz */

                /* 掩蔽该谐波区域 */
                int32_t hm0 = (int32_t)bi - (int32_t)MASK_HALF;
                int32_t hm1 = (int32_t)bi + (int32_t)MASK_HALF;
                if (hm0 < 3) hm0 = 3;
                if (hm1 >= FFT_LEN / 2) hm1 = FFT_LEN / 2 - 1;
                for (int32_t b = hm0; b <= hm1; b++) s1[b].real = 0.0f;
            }

            /* ---- 收集所有峰值, 按频率升序 ---- */
            {
                all_cnt = 0;
                all_freq[all_cnt] = fund_freq_khz;
                all_amp[all_cnt]  = fund_amp_mv;
                all_cnt++;

                for (uint8_t n = 0; n < 2; n++)
                {
                    if (harm_amp[n] >= MIN_AMP_MV)
                    {
                        all_freq[all_cnt] = harm_freq[n];
                        all_amp[all_cnt]  = harm_amp[n];
                        all_cnt++;
                    }
                }

                /* 冒泡排序 (按频率升序) */
                for (uint8_t i = 0; i < all_cnt; i++)
                {
                    for (uint8_t j = i + 1; j < all_cnt; j++)
                    {
                        if (all_freq[j] < all_freq[i])
                        {
                            float tf = all_freq[i]; all_freq[i] = all_freq[j]; all_freq[j] = tf;
                            float ta = all_amp[i];  all_amp[i]  = all_amp[j];  all_amp[j]  = ta;
                        }
                    }
                }

#if Print_screen
                /* ---- Vpp & Vrms 计算 (FFT分量合成, 相位=0) ---- */
                {
                    float sum_amp_sq = 0.0f;        /* Sigma(Vpk^2) for RMS */
                    uint8_t n;
                    for (n = 0; n < all_cnt; n++)
                    {
                        sum_amp_sq += all_amp[n] * all_amp[n];
                    }
                    /* Vrms = sqrt(Sigma Vpk^2 / 2) [mV] */
                    float vrms = sqrtf(sum_amp_sq / 2.0f);

                    /* Vpp: 重构波形 (相位=0), 找1周期内最大值 */
                    float f0_Hz = all_freq[0] * 1000.0f;
                    float T = 1.0f / f0_Hz;
                    float vmax = 0.001f;
                    {
                        uint16_t k;
                        for (k = 0; k < 200; k++)
                        {
                            float t = T * k / 200.0f;
                            float y = 0.0f;
                            for (n = 0; n < all_cnt; n++)
                            {
                                y += all_amp[n] * sinf(2.0f * PI * all_freq[n] * 1000.0f * t);
                            }
                            if (y > vmax) vmax = y;
                        }
                    }
                    float vpp = 2.0f * vmax;        /* 对称, 相位=0 */

                    /* 8次滑动平均 */
                    vpp_acc  += vpp;
                    vrms_acc += vrms;
                    meas_cnt++;

                    if (meas_cnt >= 8)
                    {
                        float vpp_avg  = vpp_acc  / 8.0f;
                        float vrms_avg = vrms_acc / 8.0f;

                        /* 串口屏文本控件 (0xFF 分隔符 = 淘晶驰协议) */
                        printf("t3.txt=\"%.1f mVpp\"%c%c%c", vpp_avg, 0xFF, 0xFF, 0xFF);
                        printf("t4.txt=\"%.1f mV\"%c%c%c", vrms_avg, 0xFF, 0xFF, 0xFF);
                        printf("t5.txt=\"%.1f KHz\"%c%c%c", all_freq[0], 0xFF, 0xFF, 0xFF);

                        /* t9 = 基波幅度 (mVpp) */
                        printf("t9.txt=\"%.1fmVpp\"%c%c%c", 2.0f * all_amp[0], 0xFF, 0xFF, 0xFF);

                        /* t7, t10 = 谐波1 频率/幅度 */
                        if (all_cnt >= 2)
                        {
                            printf("t7.txt=\"%.1fKhz\"%c%c%c", all_freq[1], 0xFF, 0xFF, 0xFF);
                            printf("t10.txt=\"%.1fmVpp\"%c%c%c", 2.0f * all_amp[1], 0xFF, 0xFF, 0xFF);
                        }

                        /* t8, t11 = 谐波2 频率/幅度 (没有则清空) */
                        if (all_cnt >= 3)
                        {
                            printf("t8.txt=\"%.1fKhz\"%c%c%c", all_freq[2], 0xFF, 0xFF, 0xFF);
                            printf("t11.txt=\"%.1fmVpp\"%c%c%c", 2.0f * all_amp[2], 0xFF, 0xFF, 0xFF);
                        }
                        else
                        {
                            printf("t8.txt=\"\"%c%c%c", 0xFF, 0xFF, 0xFF);
                            printf("t11.txt=\"\"%c%c%c", 0xFF, 0xFF, 0xFF);
                        }

                        vpp_acc  = 0.0f;
                        vrms_acc = 0.0f;
                        meas_cnt = 0;
                    }
                }
#endif
            }
        }

        /* ---- 6.3.5 波形/频谱输出 (按键或命令触发) ---- */
        RX_Change();
        if (waveform_request)
        {
            Send_Waveform(waveform_request);
            waveform_request = 0;
        }

        delay_ms(100);
    }
}

/* ============================================================================
 *  七、A/D 原始值 -> 电压转换
 *
 *  AD9226: 12-bit, 输入范围 +/-5V (差分 10Vpp)
 *  code 0     -> -5.0V
 *  code 2048  ->  0.0V
 *  code 4095  -> +5.0V
 * ============================================================================ */

void AD9226_ConvertToVoltage(uint16_t *adc_buf, float *vol_buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint16_t adc_value = adc_buf[i] & 0x0FFF;
        vol_buf[i] = ((float)adc_value / 4095.0f) * 10.0f - 5.0f;
    }
}

/* ============================================================================
 *  八、波形 & 频谱 串口屏输出 (淘晶驰协议)
 * ============================================================================ */

/**
 * @brief   发送波形数据到淘晶驰屏幕 (s0 通道)
 * @param   mode: 1=单周期, 3=三周期
 * @note    采样率 Fs = 8MHz
 *          归一化: 屏幕中心=130, 峰值=235, 谷值=25
 */
void Send_Waveform(uint8_t mode)
{
    const uint16_t SCR_W = 800;                 /* 屏幕宽度 */
    const float    CENTER = 130.0f;             /* (235+25)/2 */
    const float    RANGE  = 105.0f;             /* 235-130 */

    if (all_cnt == 0) return;

    /* 基波周期 T = 1/f0 (f0 in Hz) */
    float f0_Hz = all_freq[0] * 1000.0f;
    float T = 1.0f / f0_Hz;
    float total_time = mode * T;                /* 1T or 3T */
    float dt = total_time / (float)SCR_W;       /* 每像素时间步长 */

    /* ---- 1. 清除波形通道 ---- */
    printf("cle s0.id,0%c%c%c", 0xFF, 0xFF, 0xFF);
    delay_ms(50);

    /* ---- 2. Pass 1: 找最大 |y| 用于归一化 ---- */
    float y_max = 0.001f;
    {
        uint16_t i;
        for (i = 0; i < SCR_W; i++)
        {
            float t = i * dt;
            float y = 0.0f;
            uint8_t n;
            for (n = 0; n < all_cnt; n++)
            {
                y += all_amp[n] * sinf(2.0f * PI * all_freq[n] * 1000.0f * t);
            }
            float abs_y = fabsf(y);
            if (abs_y > y_max) y_max = abs_y;
        }
    }

    /* ---- 3. Pass 2: 发送归一化后的点 ---- */
    {
        uint16_t i;
        for (i = 0; i < SCR_W; i++)
        {
            float t = i * dt;
            float y = 0.0f;
            uint8_t n;
            for (n = 0; n < all_cnt; n++)
            {
                y += all_amp[n] * sinf(2.0f * PI * all_freq[n] * 1000.0f * t);
            }

            int32_t screen_val = (int32_t)(CENTER + (y / y_max) * RANGE);
            if (screen_val > 235) screen_val = 235;
            if (screen_val < 25)  screen_val = 25;

            printf("add s0.id,0,%d%c%c%c", (uint8_t)screen_val, 0xFF, 0xFF, 0xFF);

            if ((i % 20) == 19) delay_ms(2);
        }
    }

    /* ---- 4. 绘制频谱 (s1 通道) ---- */
    Send_Spectrum();
}

/**
 * @brief   发送幅度谱到淘晶驰屏幕 (s1 通道)
 * @note    数据来源: spectrum_mag[] (Flat-top FFT, 掩蔽前)
 *          Fs/2 = 4MHz, 8192 bins -> 800像素
 */
void Send_Spectrum(void)
{
    const uint16_t SCR_W = 800;
    const float    FS_HALF_KHZ = 4000.0f;       /* Fs/2 = 8MHz/2 = 4MHz */

    /* ---- 1. 清除 s1 ---- */
    printf("cle s1.id,0%c%c%c", 0xFF, 0xFF, 0xFF);
    delay_ms(50);

    /* ---- 2. 找最大幅度 ---- */
    float max_amp = 0.001f;
    {
        uint8_t n;
        for (n = 0; n < all_cnt; n++)
        {
            if (all_amp[n] > max_amp) max_amp = all_amp[n];
        }
    }

    /* ---- 3. 构建谱线: 只在峰值位置画线 ---- */
    {
        uint8_t line[800];
        uint16_t i;

        for (i = 0; i < SCR_W; i++) line[i] = 0;

        {
            uint8_t n;
            for (n = 0; n < all_cnt; n++)
            {
                int32_t px = (int32_t)(all_freq[n] / FS_HALF_KHZ * SCR_W + 0.5f);
                if (px < 0) px = 0;
                if (px >= SCR_W) px = SCR_W - 1;

                int32_t h = (int32_t)(all_amp[n] / max_amp * 160.0f + 0.5f);
                if (h > 160) h = 160;
                if (h < 0)   h = 0;

                line[px] = (uint8_t)h;
            }
        }

        for (i = 0; i < SCR_W; i++)
        {
            printf("add s1.id,0,%d%c%c%c", line[i], 0xFF, 0xFF, 0xFF);
            if ((i % 20) == 19) delay_ms(2);
        }
    }
}

/* ============================================================================
 *  九、串口屏按键命令处理 (USART3)
 *
 *  协议: 'd' + 命令号 + 'f'
 *    49('1'): 单周期波形
 *    50('2'): 三周期波形
 *    51('3'): 保留
 * ============================================================================ */

void RX_Change(void)
{
    /* 检查是否收到完整帧 (0x8000 = 帧完成) */
    if ((g_usart_rx_sta_2 & 0x8000) == 0) return;

    if (g_usart_rx_buf_2[0] == 100)             /* 'd' = header */
    {
        if (g_usart_rx_buf_2[1] == 49)          /* '1' -> 单周期 */
        {
            Send_Waveform(1);
        }
        else if (g_usart_rx_buf_2[1] == 50)     /* '2' -> 三周期 */
        {
            Send_Waveform(3);
        }
        else if (g_usart_rx_buf_2[1] == 51)     /* '3' -> 保留 */
        {
        }
    }
    Rx_Delete();
}

void Rx_Delete(void)
{
    for (uint8_t i = 0; i < 10; i++) g_usart_rx_buf_2[i] = 0;
    g_usart_rx_sta_2 = 0;
}

/* ============================================================================
 *  [EOF]
 * ============================================================================ */
