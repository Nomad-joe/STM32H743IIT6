/**
 ****************************************************************************************************
 * @file        iir.c
 * @brief       IIR滤波器实现（双线性变换法）
 * @attention   
 *   输入：SecondOrderTF 传递函数参数 + 采样频率
 *   输出：IIR_Filter 离散滤波器系数
 *   方法：双线性变换 s = 2/T * (1-z^-1)/(1+z^-1)
 ****************************************************************************************************
 */

#include "./BSP/IIR/my_iir.h"
#include "./SYSTEM/usart/usart.h"
#include <stdio.h>
#include <stdlib.h>

#define PI 3.1415926
/**
 * @brief 从拟合的传递函数设计IIR滤波器（双线性变换法）
 * @param tf: 拟合得到的s域传递函数参数
 * @param sample_freq: ADC采样频率 (Hz)
 * @param filter: 输出的IIR滤波器系数（z域）
 * @return 0:成功, -1:失败
 */
int IIR_Design_From_TF(SecondOrderTF* tf, float sample_freq, IIR_Filter* filter)
{
    if (tf == NULL || filter == NULL || sample_freq <= 0)
        return -1;
    
    double B0 = tf->B0;
    double B1 = tf->B1;
    double B2 = tf->B2;
    double A1 = tf->A1;
    double A2 = tf->A2;
    
    // 对于低通滤波器，B0和B1通常接近0，强制为0提高数值稳定性
    if (fabs(B0) < 1e-6) B0 = 0.0;
    if (fabs(B1) < 1e-6) B1 = 0.0;
    
    printf("\r\n===== IIR Filter Design (Bilinear Transform) =====\r\n");
    printf("Input s-domain TF:\r\n");
    printf("  B0=%.6e, B1=%.6e, B2=%.6e\r\n", B0, B1, B2);
    printf("  A1=%.6e, A2=%.6e\r\n", A1, A2);
    printf("Sample frequency: %.0f Hz\r\n", sample_freq);
    
    // 双线性变换公式:
    // s = (2/T) * (1 - z^-1) / (1 + z^-1)
    // 令 K = 2/T = 2 * fs
    
    double T = 1.0 / (double)sample_freq;
    double K = 2.0 / T;
    double K2 = K * K;
    
    // 计算公分母 D = K^2 + A1*K + A2
    double D = K2 + A1 * K + A2;
    
    if (fabs(D) < 1e-15) {
        printf("ERROR: Denominator too small (D=%.6e)!\r\n", D);
        return -1;
    }
    
    // 计算z域系数
    // H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    filter->b0 = (float)((B0 * K2 + B1 * K + B2) / D);
    filter->b1 = (float)((-2.0 * B0 * K2 + 2.0 * B2) / D);
    filter->b2 = (float)((B0 * K2 - B1 * K + B2) / D);
    filter->a1 = (float)((-2.0 * K2 + 2.0 * A2) / D);
    filter->a2 = (float)((K2 - A1 * K + A2) / D);
    
    // 初始化延迟状态
    IIR_Reset(filter);
    
    printf("\r\nOutput z-domain coefficients:\r\n");
    printf("  b0 = %.10f\r\n", filter->b0);
    printf("  b1 = %.10f\r\n", filter->b1);
    printf("  b2 = %.10f\r\n", filter->b2);
    printf("  a1 = %.10f\r\n", filter->a1);
    printf("  a2 = %.10f\r\n", filter->a2);
    
    // 检查滤波器稳定性
    // 对于分母 1 + a1*z^-1 + a2*z^-2 = 0
    // 即 z^2 + a1*z + a2 = 0
    double discriminant = (double)filter->a1 * filter->a1 - 4.0 * filter->a2;
    
    if (discriminant >= 0.0) {
        // 实极点
        double p1 = (-(double)filter->a1 + sqrt(discriminant)) / 2.0;
        double p2 = (-(double)filter->a1 - sqrt(discriminant)) / 2.0;
        printf("Poles: p1=%.6f, p2=%.6f\r\n", p1, p2);
        
        if (fabs(p1) >= 1.0 || fabs(p2) >= 1.0) {
            printf("*** WARNING: UNSTABLE FILTER! Poles outside unit circle. ***\r\n");
            return -1;
        } else {
            printf("Stability: OK (poles inside unit circle)\r\n");
        }
    } else {
        // 复极点
        double real_part = -(double)filter->a1 / 2.0;
        double imag_part = sqrt(-discriminant) / 2.0;
        double magnitude = sqrt(real_part * real_part + imag_part * imag_part);
        printf("Poles: %.6f +/- j%.6f (magnitude=%.6f)\r\n", real_part, imag_part, magnitude);
        
        if (magnitude >= 1.0) {
            printf("*** WARNING: UNSTABLE FILTER! Poles outside unit circle. ***\r\n");
            return -1;
        } else {
            printf("Stability: OK (poles inside unit circle)\r\n");
        }
    }
    
    // 验证频率响应（s域 vs z域对比）
    printf("\r\nFrequency Response Verification:\r\n");
    printf("  Freq(Hz)    s-Gain(dB)   z-Gain(dB)   Error(dB)\r\n");
    printf("  ---------   ----------   ----------   ---------\r\n");
    
    float test_freqs[] = {100.0f, 500.0f, 1000.0f, 1500.0f, 2000.0f, 3000.0f};
    for (int i = 0; i < 6; i++) {
        float f = test_freqs[i];
        
        // s域频率响应
        double w = 2.0 * PI * f;
        double w2 = w * w;
        double num_real_s = -B0 * w2 + B2;
        double num_imag_s = B1 * w;
        double den_real_s = -w2 + A2;
        double den_imag_s = A1 * w;
        double mag_s = sqrt(num_real_s*num_real_s + num_imag_s*num_imag_s) /
                       sqrt(den_real_s*den_real_s + den_imag_s*den_imag_s);
        double gain_s = 20.0 * log10(mag_s);
        
        // z域频率响应 (代入 z = e^(jωT))
        double omega = 2.0 * PI * f / sample_freq;
        double cos_w = cos(omega);
        double cos_2w = cos(2.0 * omega);
        double sin_w = sin(omega);
        double sin_2w = sin(2.0 * omega);
        
        double num_real_z = filter->b0 + filter->b1 * cos_w + filter->b2 * cos_2w;
        double num_imag_z = -filter->b1 * sin_w - filter->b2 * sin_2w;
        double den_real_z = 1.0 + filter->a1 * cos_w + filter->a2 * cos_2w;
        double den_imag_z = -filter->a1 * sin_w - filter->a2 * sin_2w;
        double mag_z = sqrt(num_real_z*num_real_z + num_imag_z*num_imag_z) /
                       sqrt(den_real_z*den_real_z + den_imag_z*den_imag_z);
        double gain_z = 20.0 * log10(mag_z);
        
        printf("  %-10.0f  %-10.2f   %-10.2f   %-10.4f\r\n", 
               f, gain_s, gain_z, gain_z - gain_s);
    }
    
    printf("===== IIR Design Complete =====\r\n\r\n");
    
    return 0;
}

/**
 * @brief IIR滤波器处理函数
 * @param src: 输入ADC原始数据 (12位: 0~4095)
 * @param dst: 输出DAC数据 (14位: 0~16383)
 * @param len: 数据长度
 * @param filter: IIR滤波器结构体指针
 * 
 * @note  信号映射:
 *        ADC 12位 (0~4095) -> 电压 0~3.3V
 *        电压 0~3.3V -> DAC电压 -2.5V~+2.5V
 *        DAC电压 -> DAC 14位 (0~16383)
 */
void IIR_Filter_Process(uint16_t *src, uint16_t *dst, uint32_t len, IIR_Filter* filter)
{
    for(uint32_t i = 0; i < len; i++)
    {
        // ADC 12位数字量 -> 电压 0~3.3V
        float input_voltage = (float)src[i] * 3.3f / 4095.0f;
        
        // 映射到DAC输入范围 -2.5V ~ +2.5V
        float x = (input_voltage / 3.3f) * 5.0f - 2.5f;
        
        // IIR差分方程:
        // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        float y = filter->b0 * x + 
                  filter->b1 * filter->x1 + 
                  filter->b2 * filter->x2 - 
                  filter->a1 * filter->y1 - 
                  filter->a2 * filter->y2;
        
        // 更新延迟状态
        filter->x2 = filter->x1;
        filter->x1 = x;
        filter->y2 = filter->y1;
        filter->y1 = y;
        
        // 限幅保护（防止溢出）
        if (y > 2.5f) y = 2.5f;
        if (y < -2.5f) y = -2.5f;
        
        // 滤波后电压 -> DAC 14位数字量 (0~16383)
        dst[i] = (uint16_t)(((y + 2.5f) / 5.0f) * 16383.0f + 0.5f);
    }
}

/**
 * @brief 重置IIR滤波器状态
 * @param filter: IIR滤波器结构体指针
 */
void IIR_Reset(IIR_Filter* filter)
{
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

