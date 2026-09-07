#ifndef __FFT_H
#define __FFT_H

#include "./SYSTEM/sys/sys.h"


#include "./CMSIS/DSP/Include/arm_math.h"
#include <math.h>
#include <stdint.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// FFT����
#define   MAX_FFT_N		 32768	

struct  compx 
{
	float32_t real, imag;
};   

void InitTableFFT(uint32_t n);
void InitWindowsFFT(uint32_t n);
void cfft(struct compx *_ptr, uint32_t FFT_N );

/* 预计算窗系数 (SDRAM, 初始化后只读) */
extern float32_t HannWindow[MAX_FFT_N];
extern float32_t FlatTopWindow[MAX_FFT_N];

#endif
