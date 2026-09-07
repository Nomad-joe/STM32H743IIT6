/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V3.0
 * @brief       AD9226 16MHz PWM DMA + 32768点自研FFT + 电赛双窗FFT/谐波分析
 *
 *  硬件连接:
 *    PH9  (TIM12_CH2) -> AD9226 CLK  (16MHz PWM)
 *    PI0~PI11         -> AD9226 D0~D11 (12-bit 并行数据)
 *    PB10/PB11        -> USART3 淘晶驰屏幕
 *
 *  架构:
 *    TIM12_CH2 16MHz PWM -> AD9226 CLK
 *    TIM12_CH1 OC1REF -> TRGO -> DMAMUX Generator(FALLING) -> DMA1_Stream0
 *    DMA Normal 模式: 填满后自动停止 -> 主循环检测 -> 停 PWM -> FFT -> 重开
 *
 *  FFT: 自研32768点基-2 DIT (参考工程), 替换 CMSIS-DSP arm_cfft_f32
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

#define FFT_LEN             32768                   /* FFT 点数 (2的幂)              */
#define SETTLING_SKIP       10                      /* 丢弃前 N 个采样点(ADC建立时间) */
#define SAMPLE_RATE_HZ      16000000.0f             /* TIM12 16MHz PWM 采样率        */

/* 抗干扰: 500KHz 以上频谱强制置零 (bin 1055 ≈ 520KHz 起) */
#define INTERFERENCE_CUTOFF_BIN  1055
#define AMP_CAL                 0.9901f  /* 幅度校准系数 (实测偏大1.01倍) */

/* ============================================================================
 *  二、全局缓冲区
 * ============================================================================ */

/* DMA 采集缓冲区 (含前导裕量) */
uint16_t DMA_Buffer[FFT_LEN + SETTLING_SKIP] = {0};

/* 电压转换缓冲区 */
float AD9226_Voltage[FFT_LEN + SETTLING_SKIP];

/* FFT: 自研复数结构体数组 (32768点) */
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
float Interf_Freq_MHz = 0.0f;   /* 1~7MHz 干扰信号频率 */
float Interf_Amp_mVpp = 0.0f;   /* 1~7MHz 干扰信号幅度 (mVpp) */
float Interf_Raw_Mag = 0.0f;    /* 干扰 bin 原始 FFT 幅度 */
uint32_t Interf_Raw_Bin = 0;    /* 干扰 bin 索引 */
float Finally_Freq;             /* 估计频率 (KHz)      */
float Finally_Amp;              /* 估计幅度 (mVpp)     */
float FFT_Amp;                  /* 峰值幅度 (raw)      */

volatile uint8_t waveform_request = 0;  /* 0=none, 1=1周期, 3=3周期 */

/* FFT 结果: 最多4个峰值 (基波 + 3谐波), 按频率升序 */
float all_freq[4], all_amp[4];
uint8_t all_order[4];  /* 谐波次数: 基波=1, 从 bin 索引比推算, 不用频率除法 */
uint8_t all_cnt;

/* 8次滑动平均累加器 (Vpp/Vrms) — 优化: 8→4, 响应速度翻倍 */
float vpp_acc = 0.0f, vrms_acc = 0.0f;
uint8_t meas_cnt = 0;
#define MEAS_AVG_N  4   /* 滑动平均次数 (原8, 优化为4: 2.4s→1.2s) */

/* 保存的幅度谱 (Flat-top FFT, 谐波掩蔽前) — SDRAM 绝对地址 */
/* HannWindow@0xC0840000(128KB) FlatTopWindow@0xC0880000(128KB) spectrum@0xC08A0000(64KB) */
float spectrum_mag[FFT_LEN / 2] __attribute__((at(0xC08A0000)));

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
    InitTableFFT(32768);                        /* SDRAM 就绪后生成 FFT 查表  */
    InitWindowsFFT(32768);                      /* 预计算 Hann + Flat-top 窗   */
    key_init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* ---- 6.2 外设初始化 ---- */
    MX_GPIO_Init();                             /* PI0~PI11 配置为输入       */
    MX_USART3_UART_Init(115200);                /* PB10/PB11 -> 淘晶驰屏幕   */
    MX_TIM12_Init();                            /* PH9 PWM 16MHz + DMA触发   */
    MX_DMA_Init();                              /* DMA1_Stream0 + DMAMUX Gen */

    /* 注册 DMA 完成回调 */
    HAL_DMA_RegisterCallback(&hdma_dma_generator0,
                             HAL_DMA_XFER_CPLT_CB_ID,
                             AD9226_DMA_CpltCallback);
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /* ---- 6.3 主循环 ---- */
    while (1)
    {   delay_ms(200);
        /* ----- 6.3.1 启动 PWM + DMA 采集 ----- */
        g_dma_done = 0;

        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);      /* PH9: AD9226 CLK      */
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);      /* DMA 触发源           */
        HAL_DMA_Start_IT(&hdma_dma_generator0,
                         (uint32_t)&GPIOI->IDR,          /* 源: PI口输入寄存器   */
                         (uint32_t)DMA_Buffer,            /* 目标: 缓冲区         */
                         FFT_LEN + SETTLING_SKIP);        /* 长度: 32768+10       */

        /* ----- 6.3.2 等待 DMA 完成 (带超时, 紧凑轮询) ----- */
        /* ★优化: 去掉 delay_ms(1), DMA 2ms即可完成, 毫秒级轮询浪费响应时间 */
        uint32_t timeout = 0;
        while (!g_dma_done && timeout < 1000000)
        {
            __NOP();
            timeout++;
        }

        /* ----- 6.3.3 停采集 ----- */
        HAL_DMA_Abort(&hdma_dma_generator0);
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);

        if (!g_dma_done) continue;                      /* 超时重试 */

        /* DMA 写入 SRAM, CPU Cache 可能持有旧数据，必须刷新 */
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)DMA_Buffer,
                                           (FFT_LEN + SETTLING_SKIP) * sizeof(uint16_t));

        /* ----- 6.3.4 原始数据 -> 电压 ----- */
        AD9226_ConvertToVoltage(DMA_Buffer, AD9226_Voltage, FFT_LEN + SETTLING_SKIP);

        /* ================================================================
         *  PASS 1: Hann 窗 -> cfft -> 频率估计
         *  Hann窗: 主瓣窄, 频率分辨率好
         *  ★优化: 预计算窗系数 → 每周期省 32768 次 arm_cos_f32 (~8ms)
         * ================================================================ */
        for (uint16_t i = 0; i < FFT_LEN; i++)
        {
            s1[i].real = AD9226_Voltage[i + SETTLING_SKIP] * HannWindow[i];
            s1[i].imag = 0.0f;
        }

        cfft(s1, FFT_LEN);

        /* ---- 计算幅度谱 ---- */
        for (uint16_t i = 0; i < FFT_LEN; i++)
        {
            arm_sqrt_f32(s1[i].real * s1[i].real + s1[i].imag * s1[i].imag,
                         &s1[i].real);
        }

        /* 抑制直流 bin */
        s1[0].real = 0.0f;
        s1[1].real = 0.0f;
        s1[2].real = 0.0f;

        /* 抗干扰: 500KHz 以上频谱直接置零 */
        for (uint16_t i = INTERFERENCE_CUTOFF_BIN; i < FFT_LEN / 2; i++)
            s1[i].real = 0.0f;

        /* ================================================================
         *  Hann 多峰检测 + 能量重心插值 (替换 Jacobsen)
         *  能量重心: freq = Σ(bin_i * mag_i²) / Σ(mag_i²) * BIN
         *  精度 ~0.02 bin ≈ 10 Hz, 对所有分量(基波+谐波)统一处理
         * ================================================================ */
        {
            const float BIN_WIDTH = SAMPLE_RATE_HZ / (float)FFT_LEN;  /* 488.28125 Hz */

            /* 噪声估计 + 自适应阈值 */
            float mag_max = 0.0f, mag_sum = 0.0f;
            uint32_t mag_n = 0;
            uint32_t i;
            for (i = 3; i < FFT_LEN / 2; i++)
            {
                if (s1[i].real > mag_max) mag_max = s1[i].real;
                mag_sum += s1[i].real;
                mag_n++;
            }
            float noise_floor = (mag_sum / (float)mag_n) * 3.0f;
            float thresh = noise_floor;
            if (mag_max * 0.05f > thresh) thresh = mag_max * 0.05f;

            /* 5点局部最大 + 阈值 → 多峰检测 (最多4个) */
            all_cnt = 0;
            for (i = 3; i < FFT_LEN / 2 - 2 && all_cnt < 4; i++)
            {
                if (s1[i].real > s1[i-1].real && s1[i].real > s1[i-2].real &&
                    s1[i].real > s1[i+1].real && s1[i].real > s1[i+2].real &&
                    s1[i].real > thresh)
                {
                    /* ★ 能量重心法: 平方幅度加权, Hann窗最大似然估计 */
                    if (i >= 1 && i < FFT_LEN/2 - 1)
                    {
                        float m0 = s1[i-1].real; m0 *= m0;
                        float m1 = s1[i].real;   m1 *= m1;
                        float m2 = s1[i+1].real; m2 *= m2;
                        float idx_c = ((i-1)*m0 + i*m1 + (i+1)*m2) / (m0+m1+m2);
                        all_freq[all_cnt] = idx_c * BIN_WIDTH / 1000.0f;   /* -> KHz */
                    }
                    else
                    {
                        all_freq[all_cnt] = i * BIN_WIDTH / 1000.0f;
                    }
                    all_amp[all_cnt] = (float)i;  /* 暂存 bin 索引, 供 Pass 2 查找 */
                    all_cnt++;
                }
            }
        }

        /* 更新全局变量 (向后兼容) */
        if (all_cnt > 0)
        {
            Index        = (uint32_t)all_amp[0];     /* 基波 bin */
            Finally_Index = all_freq[0] * 1000.0f / (SAMPLE_RATE_HZ / (float)FFT_LEN);
            Finally_Freq  = all_freq[0];              /* 基波频率 (KHz) */
        }
        else
        {
            Index = 3;
            Finally_Index = 3.0f;
            Finally_Freq  = 0.0f;
        }

        /* ================================================================
         *  PASS 2: Flat-top 窗 -> cfft -> 幅度估计 + 谐波分析
         *  Flat-top: scalloping loss < 0.005 dB, 幅度精度极高
         *  ★优化: 预计算窗系数 → 每周期省 131072 次 arm_cos_f32 (~32ms)
         * ================================================================ */
        {
            for (uint16_t i = 0; i < FFT_LEN; i++)
            {
                s1[i].real = AD9226_Voltage[i + SETTLING_SKIP] * FlatTopWindow[i];
                s1[i].imag = 0.0f;
            }
        }

        cfft(s1, FFT_LEN);

        /* 手动计算幅度 + 保存相位 (.real=幅度 .imag=相位 rad) */
        for (uint16_t i = 0; i < FFT_LEN; i++)
        {
            float re = s1[i].real;
            float im = s1[i].imag;
            arm_sqrt_f32(re * re + im * im, &s1[i].real);
            s1[i].imag = atan2f(im, re);
        }

        /* 抑制直流 & 近直流 bin */
        s1[0].real = 0.0f;
        s1[1].real = 0.0f;
        s1[2].real = 0.0f;
        s1[3].real = 0.0f;

        /* ===== 干扰信号检测 (1MHz ~ 7MHz, 在清零之前, 只存原始值) ===== */
        {
            const uint32_t INTERF_START = 2048;    /* 1MHz / 488.28Hz */
            const uint32_t INTERF_END   = 14336;   /* 7MHz / 488.28Hz */

            Interf_Raw_Mag = 0.0f;
            Interf_Raw_Bin = 0;
            Interf_Freq_MHz = 0.0f;
            Interf_Amp_mVpp = 0.0f;

            for (uint32_t ii = INTERF_START; ii < INTERF_END && ii < FFT_LEN / 2; ii++)
            {
                if (s1[ii].real > Interf_Raw_Mag)
                {
                    Interf_Raw_Mag = s1[ii].real;
                    Interf_Raw_Bin = ii;
                }
            }
        }

        /* 抗干扰: 500KHz 以上频谱直接置零 */
        for (uint16_t i = INTERFERENCE_CUTOFF_BIN; i < FFT_LEN / 2; i++)
            s1[i].real = 0.0f;

        /* 保存干净频谱 (谐波掩蔽前) */
        for (uint16_t si = 0; si < FFT_LEN / 2; si++)
            spectrum_mag[si] = s1[si].real;

        /* ---- 在 Hann 找到的每个 bin 处, 从 Flat-top 谱提取精确幅值 ----
         *  频率已在 Pass 1 用 Hann 能量重心法精确测定 (~10Hz),
         *  此处 Flat-top 只负责提取幅度 (通带平坦 ±0.01dB)
         * ---- */
        {
            const float FT_NORM  = 2000.0f / ((float)FFT_LEN * 0.21557895f) * AMP_CAL;
            const float MIN_AMP_MV = 3.0f;

            float fund_raw_mag = 0.0f;
            uint32_t fund_bin = (uint32_t)all_amp[0];   /* 基波 bin (Hann) */

            /* 在每个 Hann 峰位置提取 Flat-top 幅度 */
            for (uint8_t i = 0; i < all_cnt; i++)
            {
                uint32_t bi = (uint32_t)all_amp[i];     /* Hann 找到的 bin */
                if (bi < FFT_LEN / 2)
                {
                    float ft_mag = s1[bi].real;         /* Flat-top 幅度 (raw) */
                    all_amp[i] = ft_mag * FT_NORM;      /* 校准为 mV (peak) */

                    if (i == 0)
                    {
                        fund_raw_mag = ft_mag;
                        FFT_Amp      = ft_mag;
                        Finally_Amp  = all_amp[0];
                    }

                    /* 谐波次数: bin 索引整数比推算 */
                    all_order[i] = (uint8_t)((bi + fund_bin / 2) / fund_bin);
                    if (all_order[i] < 1) all_order[i] = 1;
                }
                else
                {
                    all_amp[i]   = 0.0f;
                    all_order[i] = 1;
                }
            }

            /* 剔除 <3mV 的噪声伪峰 */
            {
                uint8_t valid = 0;
                for (uint8_t i = 0; i < all_cnt; i++)
                {
                    if (all_amp[i] >= MIN_AMP_MV)
                    {
                        if (valid != i)
                        {
                            all_freq[valid]  = all_freq[i];
                            all_amp[valid]   = all_amp[i];
                            all_order[valid] = all_order[i];
                        }
                        valid++;
                    }
                }
                all_cnt = valid;
            }

            /* 更新基波变量 (噪声滤波后可能重排) */
            if (all_cnt > 0)
            {
                Index       = (uint32_t)all_amp[0];
                Finally_Freq = all_freq[0];
                Finally_Amp  = all_amp[0];
                FFT_Amp      = fund_raw_mag;
            }

            /* 干扰信号校准: 与基波共用同一套校准系数 */
            if (Interf_Raw_Bin > 0 && fund_raw_mag > 0.0f)
            {
                float calib = Finally_Amp / fund_raw_mag;
                float interf_mVpk = Interf_Raw_Mag * calib;
                float interf_mVpp = interf_mVpk * 2.0f;
                if (interf_mVpp >= 10.0f)
                {
                    Interf_Freq_MHz = (float)Interf_Raw_Bin * 488.28125f / 1000000.0f;
                    Interf_Amp_mVpp = interf_mVpp;
                }
            }

            /* >100.4KHz 高频段幅度修正 */
            for (uint8_t i = 0; i < all_cnt; i++)
            {
                if (all_freq[i] > 100.4f) all_amp[i] *= 0.995063f;
            }

            /* 按频率升序排列 (冒泡, 同步交换 freq/amp/order) */
            for (uint8_t i = 0; i < all_cnt; i++)
            {
                for (uint8_t j = i + 1; j < all_cnt; j++)
                {
                    if (all_freq[j] < all_freq[i])
                    {
                        float tf = all_freq[i]; all_freq[i] = all_freq[j]; all_freq[j] = tf;
                        float ta = all_amp[i];  all_amp[i]  = all_amp[j];  all_amp[j]  = ta;
                        uint8_t to = all_order[i]; all_order[i] = all_order[j]; all_order[j] = to;
                    }
                }
            }

#if Print_screen
                /* ---- Vpp (波形重构寻峰) & Vrms (FFT分量合成) ---- */
                {
                    float sum_amp_sq = 0.0f;        /* Sigma(Vpk^2) for RMS */
                    uint8_t n;
                    for (n = 0; n < all_cnt; n++)
                    {
                        sum_amp_sq += all_amp[n] * all_amp[n];
                    }
                    /* Vrms = sqrt(Sigma Vpk^2 / 2) [mV] */
                    float vrms = sqrtf(sum_amp_sq / 2.0f);

                    /* === Vpp: 角度波形重建, 2000步数值寻峰 ===
                     *  参考 C语言求幅度.txt: all_amp[] 为 mVpeak,
                     *  合成 y = Σ A_n * sin(order_n * θ), θ∈[0,2π]
                     *  2000步保证多谐波叠加时峰峰值精度 ~0.1% */
                    float vpp;
                    {
                        float y_max = -1e9f, y_min = 1e9f;
                        const uint16_t steps = 2000;
                        uint16_t k;
                        for (k = 0; k < steps; k++)
                        {
                            float theta = 2.0f * PI * k / (float)steps;
                            float y = all_amp[0] * sinf(theta);
                            uint8_t hn;
                            for (hn = 1; hn < all_cnt; hn++)
                            {
                                y += all_amp[hn] * sinf((float)all_order[hn] * theta);
                            }
                            if (y > y_max) y_max = y;
                            if (y < y_min) y_min = y;
                        }
                        vpp = y_max - y_min;
                    }

                    /* N次滑动平均 (Vpp和Vrms都每周期累加) */
                    vpp_acc  += vpp;
                    vrms_acc += vrms;
                    meas_cnt++;

                    if (meas_cnt >= MEAS_AVG_N)
                    {
                        float vpp_avg  = vpp_acc  / (float)MEAS_AVG_N;
                        float vrms_avg = vrms_acc / (float)MEAS_AVG_N;

                        /* 串口屏文本控件 (0xFF 分隔符 = 淘晶驰协议) */
                        printf("t3.txt=\"%.3f mVpp\"%c%c%c", vpp_avg, 0xFF, 0xFF, 0xFF);
                        printf("t4.txt=\"%.3f mV\"%c%c%c", vrms_avg, 0xFF, 0xFF, 0xFF);
                        printf("t5.txt=\"%.3f Khz\"%c%c%c", all_freq[0], 0xFF, 0xFF, 0xFF);

                        /* t9 = 基波幅度 (mVpp) */
                        printf("t9.txt=\"%.3fmV\"%c%c%c",  all_amp[0], 0xFF, 0xFF, 0xFF);

                        /* t7, t10 = 谐波1 频率/幅度 */
                        if (all_cnt >= 2)
                        {
                            printf("t7.txt=\"%.3fKhz\"%c%c%c", all_freq[1], 0xFF, 0xFF, 0xFF);
                            printf("t10.txt=\"%.3fmV\"%c%c%c",  all_amp[1], 0xFF, 0xFF, 0xFF);
                        }

                        /* t8, t11 = 谐波2 频率/幅度 (没有则清空) */
                        if (all_cnt >= 3)
                        {
                            printf("t8.txt=\"%.3fKhz\"%c%c%c", all_freq[2], 0xFF, 0xFF, 0xFF);
                            printf("t11.txt=\"%.3fmV\"%c%c%c",  all_amp[2], 0xFF, 0xFF, 0xFF);
                        }
                        else
                        {
                            printf("t8.txt=\"\"%c%c%c", 0xFF, 0xFF, 0xFF);
                            printf("t11.txt=\"\"%c%c%c", 0xFF, 0xFF, 0xFF);
                        }

                        /* t19 = 1~7MHz 干扰信号 */
                        if (Interf_Amp_mVpp > 0.0f)
                        {
                          printf("t19.txt=\"jamming signal %.3f Mhz \"%c%c%c",
                                     Interf_Freq_MHz, 0xFF, 0xFF, 0xFF);
                        }
                        else
                        {
                            printf("t19.txt=\"\"%c%c%c", 0xFF, 0xFF, 0xFF);
                        }

                        vpp_acc  = 0.0f;
                        vrms_acc = 0.0f;
                        meas_cnt = 0;
                    }
                }
#endif
        }

        /* ---- 6.3.5 波形/频谱输出 (按键或命令触发) ---- */
        RX_Change();
        if (waveform_request)
        {
            Send_Waveform(waveform_request);
            waveform_request = 0;
        }
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
 * @note    采样率 Fs = 16MHz
 *          归一化: 屏幕中心=67.5, 峰值=120, 谷值=15
 */
void Send_Waveform(uint8_t mode)
{
    const uint16_t SCR_W = 800;                 /* 屏幕宽度 */
    const float    CENTER = 67.5f;              /* (120+15)/2 */
    const float    RANGE  = 52.5f;              /* 120-67.5 */

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
            if (screen_val > 120) screen_val = 120;
            if (screen_val < 15)  screen_val = 15;

            printf("add s0.id,0,%d%c%c%c", (uint8_t)screen_val, 0xFF, 0xFF, 0xFF);

            if ((i % 20) == 19) delay_ms(2);
        }
    }

    /* ---- 4. 绘制频谱 (s1 通道) ---- */
    Send_Spectrum();
}

/**
 * @brief   发送幅度谱到淘晶驰屏幕 (s0 通道1, 与波形共用一个控件)
 * @note    数据来源: spectrum_mag[] (Flat-top FFT, 掩蔽前)
 *          仅显示 0~500KHz 范围, 映射到 800 像素, 谱线最大高度 120
 */
void Send_Spectrum(void)
{
    const uint16_t SCR_W = 800;
    const float    FREQ_MAX_KHZ = 501.0f;        /* 有效信号上限 */

    /* ---- 1. 清除 s1 ---- */
    printf("cle s1.id,0%c%c%c", 0xFF, 0xFF, 0xFF);
    delay_ms(50);

    if (all_cnt == 0) return;

    float f0 = all_freq[0];                      /* 基波频率 (KHz) */

    /* ---- 2. 为每个检出的峰值确定谐波次数, 并找最大幅度 ---- */
    uint8_t  harm_order[4];
    float    max_amp = 0.001f;
    uint8_t  valid_cnt = 0;
    uint8_t  n;
    for (n = 0; n < all_cnt; n++)
    {
        uint8_t order = (uint8_t)roundf(all_freq[n] / f0);
        if (order < 1) order = 1;
        /* 谐波频率超过501KHz的不显示 */
        if ((float)order * f0 > FREQ_MAX_KHZ) continue;

        harm_order[valid_cnt] = order;
        valid_cnt++;
        if (all_amp[n] > max_amp) max_amp = all_amp[n];
    }

    /* ---- 3. 构建谱线: 基波在40px, n次谐波在 n*40 px ---- */
    {
        uint8_t line[800];
        uint16_t i;

        for (i = 0; i < SCR_W; i++) line[i] = 0;

        for (n = 0; n < valid_cnt; n++)
        {
            int32_t px = (int32_t)harm_order[n] * 40;
            if (px >= SCR_W) px = SCR_W - 1;

            int32_t h = (int32_t)(all_amp[n] / max_amp * 120.0f + 0.5f);
            if (h > 120) h = 120;
            if (h < 1)   h = 1;                  /* 至少1像素可见 */

            line[px] = (uint8_t)h;
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
