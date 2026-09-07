#include "./BSP/FFT/my_fft.h"
#include "./CMSIS/DSP/Include/arm_math.h"
#include "./SYSTEM/usart/usart.h"


void FFT(float* ADC_Float,float *FFT_input,float*FFT_mag,uint16_t FFT_LENGTH)//传入ADC数据 
{

	for(uint8_t i=0;i<FFT_LENGTH;i++)//增加汉宁窗，抑制频谱泄露
	{
	  ADC_Float[i] *=0.5000f*(1 - arm_cos_f32(2*PI*i /(FFT_LENGTH -1)) );
	}
  for(uint16_t i=0;i<FFT_LENGTH;i++)//转变为复信号
	{
	  FFT_input[2*i] = ADC_Float[i];
	  FFT_input[2*i +1] = 0;
	}
	 arm_cfft_instance_f32 fftInst; 
  
   arm_cfft_f32(&fftInst,FFT_input,0,1); //FFT计算
	
	 arm_cmplx_mag_f32(FFT_input,FFT_mag,FFT_LENGTH);
	 
   for(uint16_t i=0;i<FFT_LENGTH;i++)
	{
	 printf("%.3f\n",FFT_mag[i]); //打印幅度谱
	}
}


