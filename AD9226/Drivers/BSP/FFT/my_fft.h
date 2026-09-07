#ifndef __FFT_H
#define __FFT_H

#include "./SYSTEM/sys/sys.h"


#include "./CMSIS/DSP/Include/arm_math.h"
#include <math.h>
#include <stdint.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// FFT≈‰÷√
#define   MAX_FFT_N		 16384	

struct  compx 
{
	float32_t real, imag;
};   

void InitTableFFT(uint32_t n);
void cfft(struct compx *_ptr, uint32_t FFT_N );


#endif
