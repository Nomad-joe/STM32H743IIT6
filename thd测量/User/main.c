/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       DSP FFT 实验 - 全相位FFT相位差测量
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
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

#include "./BSP/AD9834/my_ad9834.h"
#include "./BSP/AD9834_2/my_ad9834_2.h"
#include "./CMSIS/DSP/Include/arm_math.h"
#include "./CMSIS/DSP/Include/arm_const_structs.h"

#define FFT_LEN  16384


uint32_t g_timeout;
uint32_t time;

uint8_t adc1_dma_complete_flag=0;

uint16_t DMA_Buffer[FFT_LEN] = {0};


uint32_t Index = 0;


float Amp_1,Amp_2,Amp_3,Amp_4,Amp_5;


struct  compx s1[MAX_FFT_N]; 

int main(void)
{  
    sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(192, 5, 2, 4);
    delay_init(480);
    usart_init(115200);
    led_init();
    mpu_memory_protection();
    sdram_init();
    key_init();

    MX_TIM3_Init();
    PeriphCommonClock_Config();
    MX_ADC1_Init();


    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);delay_ms(10);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_Buffer, FFT_LEN);
	  HAL_TIM_Base_Start(&htim3);delay_ms(15);
	
    while (1)
    { 
       while(adc1_dma_complete_flag==0){}
				 adc1_dma_complete_flag = 0;
        for(uint16_t i = 0; i < FFT_LEN; i++)// 数据采集和加窗
        {
            s1[i].real = (float)DMA_Buffer[i] * 3.3f / 65536;
            s1[i].real *= 0.5f * (1 - arm_cos_f32(2 * PI * i / (FFT_LEN - 1))); 
          
					  s1[i].imag =0;
        }
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_Buffer, FFT_LEN);
    cfft(s1,MAX_FFT_N);

	 float Vpp = 0; uint16_t index = 0;
   for(uint16_t i = 2; i < FFT_LEN/2; i++) {
    float mag = sqrtf(s1[i].real * s1[i].real + s1[i].imag * s1[i].imag);
    if(mag > Vpp) { Vpp = mag; index = i; }
}



	float Finally_index = (float)((index-1)*s1[index-1].real*s1[index-1].real + index*s1[index].real*s1[index].real + (index+1)*s1[index+1].real*s1[index+1].real) / (s1[index-1].real*s1[index-1].real + s1[index].real*s1[index].real + s1[index+1].real*s1[index+1].real);
  printf("Freq = %.2f khz\r\n",(float)Finally_index*1500.0f/16384.0f);
  
	for(uint16_t i=0;i<FFT_LEN/2;i++)
	{
	  s1[i].real = sqrt(s1[i].real * s1[i].real + s1[i].imag * s1[i].imag);
	}
	Amp_1 = sqrt(s1[(int)Finally_index-1].real * s1[(int)Finally_index-1].real + s1[(int)Finally_index].real * s1[(int)Finally_index].real+s1[(int)Finally_index+1].real * s1[(int)Finally_index+1].real);
  Amp_2 = sqrt(s1[(int)(2*Finally_index)-1].real * s1[(int)(2*Finally_index)-1].real + s1[(int)(2*Finally_index)].real * s1[(int)(2*Finally_index)].real+s1[(int)(2*Finally_index)+1].real * s1[(int)(2*Finally_index)+1].real);
  Amp_3 = sqrt(s1[(int)(3*Finally_index)-1].real * s1[(int)(3*Finally_index)-1].real + s1[(int)(3*Finally_index)].real * s1[(int)(3*Finally_index)].real+s1[(int)(3*Finally_index)+1].real * s1[(int)(3*Finally_index)+1].real);
  Amp_4 = sqrt(s1[(int)(4*Finally_index)-1].real * s1[(int)(4*Finally_index)-1].real + s1[(int)(4*Finally_index)].real * s1[(int)(4*Finally_index)].real+s1[(int)(4*Finally_index)+1].real * s1[(int)(4*Finally_index)+1].real);
  Amp_5 = sqrt(s1[(int)(5*Finally_index)-1].real * s1[(int)(5*Finally_index)-1].real + s1[(int)(5*Finally_index)].real * s1[(int)(5*Finally_index)].real+s1[(int)(5*Finally_index)+1].real * s1[(int)(5*Finally_index)+1].real);
  
	printf("Amp_1 = %.2f\r\n",1.0);
	printf("Amp_2 = %.2f\r\n",Amp_2/Amp_1);
	printf("Amp_3 = %.2f\r\n",Amp_3/Amp_1);
	printf("Amp_4 = %.2f\r\n",Amp_4/Amp_1);
	printf("Amp_5 = %.2f\r\n",Amp_5/Amp_1);	
	float Thd = sqrt(Amp_2*Amp_2 +Amp_3*Amp_3 +Amp_4*Amp_4 +Amp_5*Amp_5) / Amp_1;
	//int Idx = (int)index;
//float R1_prev2 = s1[Idx-2].real, R1_prev1 = s1[Idx-1].real, R1_curr = s1[Idx].real, R1_next1 = s1[Idx+1].real, R1_next2 = s1[Idx+2].real;
//float I1_prev2 = s1[Idx-2].imag, I1_prev1 = s1[Idx-1].imag, I1_curr = s1[Idx].imag, I1_next1 = s1[Idx+1].imag, I1_next2 = s1[Idx+2].imag;
//float w_prev2 = R1_prev2*R1_prev2 + I1_prev2*I1_prev2, w_prev1 = R1_prev1*R1_prev1 + I1_prev1*I1_prev1, w_curr = R1_curr*R1_curr + I1_curr*I1_curr, w_next1 = R1_next1*R1_next1 + I1_next1*I1_next1, w_next2 = R1_next2*R1_next2 + I1_next2*I1_next2;
//float real_1 = (R1_prev2*w_prev2 + R1_prev1*w_prev1 + R1_curr*w_curr + R1_next1*w_next1 + R1_next2*w_next2) / (w_prev2 + w_prev1 + w_curr + w_next1 + w_next2);
//float img_1  = (I1_prev2*w_prev2 + I1_prev1*w_prev1 + I1_curr*w_curr + I1_next1*w_next1 + I1_next2*w_next2) / (w_prev2 + w_prev1 + w_curr + w_next1 + w_next2);

//	 Vpp = 2*sqrt(real_1*real_1 + img_1*img_1);
//   if(Vpp>2310)Vpp /= 1.542667;        //1.5V校正
//   else if(Vpp>1150)Vpp /= 1.534667;  //750mv校正
//   else if(Vpp>604)Vpp /= 1.5125;  //400mv校正
//   else if(Vpp>105)Vpp /= 1.24;  //100mv校正
//   else if(Vpp>97)Vpp /= 1.1647;  //85mv校正
//   else if(Vpp>50)Vpp /= 1;     //65mv校正
//   else if(Vpp>10)Vpp /= 0.7;  //50mv校正
//   printf("Vpp = %.2fmv \r\n",Vpp);
	delay_ms(1000);
		
  }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        adc1_dma_complete_flag = 1;
    }
    
}

