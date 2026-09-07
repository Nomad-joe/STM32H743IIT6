#ifndef __IIR_H
#define __IIR_H

#include "./SYSTEM/sys/sys.h"


// 拟合输出的归一化二阶传递函数结构体
typedef struct
{
    double A1;
    double A2;
    double B0;
    double B1;
    double B2;
} SecondOrderTF;

// IIR滤波器结构体
typedef struct {
    float b0, b1, b2;  // 分子系数
    float a1, a2;      // 分母系数 (a0=1)
    float x1, x2;      // 输入延迟状态
    float y1, y2;      // 输出延迟状态
} IIR_Filter;

// 函数声明
int IIR_Design_From_TF(SecondOrderTF* tf, float sample_freq, IIR_Filter* filter);
void IIR_Filter_Process(uint16_t *src, uint16_t *dst, uint32_t len, IIR_Filter* filter);
void IIR_Reset(IIR_Filter* filter);


#endif

