/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       DSP FFT 实验
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
#include "./CMSIS/DSP/Include/arm_math.h"

uint32_t g_timeout;
uint32_t time;
/**
 * FFT长度，默认是1024点FFT 
 * 可选范围为: 16, 64, 256, 1024.
 */
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
	
#define FFT_LEN  4096 
uint16_t DMA_Buffer[FFT_LEN] = {0};
float ADC_float [FFT_LEN] = {0};
float FFT_input[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};
uint32_t Index = 0; //mag数组中最大值的下标
uint32_t Index2 =0;
uint32_t Index3 =0;
float Finally_Index ;
float Finally_Freq;
float Finally_Amp;
float Finally_Amp2;
float Finally_Amp3;
float Finally_Amp4;
float Finally_Amp5;
float FFT_Amp;
float DC;//直流量
//uint32_t ADC_Max_Index ;//配套使用，否则没法使用arm_max_f32函数
int main(void)
{  
    sys_cache_enable();     
	/* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
 
   MX_ADC1_Init();//初始化adc PA0
   MX_TIM3_Init();//初始化adc所需的timer 频率为500K
	 HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
	 delay_ms(20);
    timx_int_init(65535, 240 - 1);          /* 1Mhz计数频率,设置自动重载值为65536 */
  
    while (1)
    {   HAL_TIM_Base_Start(&htim3);     
       HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC开始采集，DMA开启搬运
		 	delay_ms(100);
			HAL_TIM_Base_Stop(&htim3);
			HAL_ADC_Stop_DMA(&hadc1);
			DC =0;
      for(uint16_t i=0;i<FFT_LEN;i++){DC += (float)DMA_Buffer[i] / FFT_LEN;} 
			for(uint16_t i=0;i<FFT_LEN;i++)
			{
			  ADC_float[i] = (float)DMA_Buffer[i]*3.3f/65536;
				ADC_float[i] -= DC*3.3f/65536;
				ADC_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗
			}
   for(uint16_t i=0;i<FFT_LEN;i++)//转变为复信号
			{
			 FFT_input[2*i] = ADC_float[i];
			 FFT_input[2*i+1] = 0;
			}


    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			

	 Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
	 Finally_Freq = Finally_Index*244.140625; //1.5M采样率 366.2109375  500K 122.0703125 1M 244.140625
	 Finally_Freq=Finally_Freq/1000;
			
   Finally_Amp = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1])/1.2464;
   Finally_Amp2 = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1])/1.2464;
	 Finally_Amp3 = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1])/1.2464;		
   Finally_Amp4 = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1])/1.2464;
   Finally_Amp5 = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1])/1.2464;			
	 printf("测量频率：%.4f kHz\r\n",Finally_Freq);
   printf("测量幅值: %.3f v\r\n",Finally_Amp);
			
		}
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc ==&hadc1){

	}
}


