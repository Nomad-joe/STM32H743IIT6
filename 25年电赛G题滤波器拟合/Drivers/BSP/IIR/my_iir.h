#ifndef __IIR_H
#define __IIR_H

#include "./SYSTEM/sys/sys.h"


// 归一化的二阶模拟传递函数结构体
typedef struct
{
    double A1;
    double A2;
    double B0;
    double B1;
    double B2;
} SecondOrderTF;

// IIR二阶节(biquad)滤波器结构体
typedef struct {
    float b0, b1, b2;  // 分子系数
    float a1, a2;      // 分母系数 (a0=1)
    float x1, x2;      // 输入延迟状态
    float y1, y2;      // 输出延迟状态
} IIR_Filter;

// 四阶IIR (Direct Form I)
typedef struct {
    float b[5];        // b0..b4
    float a[4];        // a1..a4
    float x[4];        // x[n-1]..x[n-4]
    float y[4];        // y[n-1]..y[n-4]
} IIR_Filter4;

// 四阶SOS (向后兼容)
typedef struct {
    IIR_Filter stage1;
    IIR_Filter stage2;
} IIR_SOS;

// 扫频数据结构体
typedef struct {
    float freq;
    float gain_db;
    float phase_deg;
} BodeData;

// 函数声明
int IIR_Design_From_TF(SecondOrderTF* tf, float sample_freq, IIR_Filter* filter);
int IIR_Design_Direct(BodeData* data, int cnt, float fs, IIR_Filter* filter);
int IIR_Design_Direct_4th(BodeData* data, int cnt, float fs, IIR_SOS* sos);

void IIR_Filter_Process(uint16_t *src, uint16_t *dst, uint32_t len, IIR_Filter* filter);
void IIR_Filter_Process_4th(uint16_t *src, uint16_t *dst, uint32_t len, IIR_Filter4* f4);
void IIR_Reset(IIR_Filter* filter);
void IIR_SOS_Reset(IIR_SOS* sos);


#endif
