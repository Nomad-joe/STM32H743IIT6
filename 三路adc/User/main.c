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

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
 	
#define FFT_LEN  4096 
uint16_t DMA_Buffer[FFT_LEN] = {0};
float ADC_float [FFT_LEN] = {0};
float ADC_float_2 [FFT_LEN] = {0};
float FFT_input[2*FFT_LEN] = {0};
float FFT_input_2[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};//存放频谱
float FFT_mag_2[FFT_LEN] ={0};
uint32_t Index = 0; //mag数组中最大值的下标
uint32_t Index2 =0;
uint32_t Index3 =0;
float Finally_Index ;
float Finally_Freq;  //准确的频率
float Finally_Amp;   //准确的幅值

float Finally_Phase; //准确的相位
float Finally_Phase_2; //准确的相位
float FFT_Amp;

float DMA_Max ;//记录最大的电压采样，用于减去直流量
float DMA_Max_2 ;
uint16_t DMA_Buffer_2[FFT_LEN] = {0};
uint16_t dma_buffer_sram4[FFT_LEN] __attribute__((section(".ARM.__at_0x38000000")));

int main(void)
{  
    sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
 
	
	MX_TIM3_Init();
  PeriphCommonClock_Config();
	MX_ADC1_Init();//PA0
	MX_ADC2_Init();//PA7
  MX_ADC3_Init();//PC3
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	delay_ms(20);

  
    while (1)
    { HAL_TIM_Base_Start(&htim3);
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC1开始采集，DMA开启搬运
			HAL_ADC_Start_DMA(&hadc2,(uint32_t*)DMA_Buffer_2,FFT_LEN);//ADC2开始采集，DMA开启搬运
      HAL_ADC_Start_DMA(&hadc3,(uint32_t*)dma_buffer_sram4,FFT_LEN);//ADC3开始采集，DMA开启搬运
		 	delay_ms(1000);
			HAL_ADC_Stop_DMA(&hadc1);
			HAL_ADC_Stop_DMA(&hadc2);
			HAL_ADC_Stop_DMA(&hadc3);
		  HAL_TIM_Base_Stop(&htim3);
			for(uint32_t i =0;i<4096;i++)
			{
			  printf("%.2f\r",(float)dma_buffer_sram4[i]*3300/65535);
			}
			delay_ms(1000);
#if  1			
      DMA_Max = 0;DMA_Max_2 = 0;
			for(uint16_t i=0;i<FFT_LEN;i++){if(DMA_Buffer[i]>DMA_Max)DMA_Max = DMA_Buffer[i];}//找出最大的电压值
			for(uint16_t i=0;i<FFT_LEN;i++){if(DMA_Buffer_2[i]>DMA_Max_2)DMA_Max_2 = DMA_Buffer_2[i];}//找出最大的电压值
			for(uint16_t i=0;i<FFT_LEN;i++)
			{
			  ADC_float[i] = (float)DMA_Buffer[i]*3.3f/65536;
				ADC_float[i] -=DMA_Max*1.65f/65536;//去除直流分量
				ADC_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗
				
			  ADC_float_2[i] = (float)DMA_Buffer_2[i]*3.3f/65536;
				ADC_float_2[i] -=DMA_Max*1.65f/65536;//去除直流分量
				ADC_float_2[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗				

				
			}
			
   for(uint16_t i=0;i<FFT_LEN;i++)//转变为复信号
			{
			 FFT_input[2*i] = ADC_float[i];
			 FFT_input[2*i+1] = 0;
				
			 FFT_input_2[2*i] = ADC_float_2[i];
			 FFT_input_2[2*i+1] = 0;
			}
			
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input_2, 0, 1);
//    arm_cmplx_mag_f32(FFT_input_2, FFT_mag_2, FFT_LEN);
//		arm_max_f32(FFT_mag_2,FFT_LEN/2,&FFT_Amp_2,&Index_2);
   		
   float real_1,real_2 =0;
	 float img_1,img_2  =0;
   float pahse_1,phase_2 = 0;	
   real_1 = FFT_input[2*Index];
	 img_1  = FFT_input[2*Index+1];
	 //printf("phase = %.2lf 度\r\n",atan2f(img_1,real_1)*180.0f/PI);	
	float phase_1 = 	atan2f(img_1,real_1)*180.0f/PI;	
   real_2 = FFT_input_2[2*Index];
	 img_2  = FFT_input_2[2*Index+1];
	 //printf("phase = %.2lf 度\r\n",atan2f(img_2,real_2)*180.0f/PI);			
   phase_2 = atan2f(img_2,real_2)*180.0f/PI;
		float diff = 	phase_2-phase_1;
		if(diff>180)diff -=360;
		if(diff<-180)  diff +=360;
		printf("相位差：%.2f 度\r\n",phase_2-phase_1);	
  


//	 Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
//	 Finally_Freq = Finally_Index*366.2109375;
//	 Finally_Freq=Finally_Freq/1000;
//			
//   Finally_Amp = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1]);
//   Finally_Amp /= 1.24832;

//	 printf("测量频率：%.2f kHz\r\n",Finally_Freq);
//   printf("测量幅值: %.3f mv\r\n",Finally_Amp);

	
#endif			
			
//		  for(uint16_t i=0;i<FFT_LEN;i++)
//			{
//			 printf("%.2f mv\r\n",(float)DMA_Buffer_2[i]*3300.0f/65535.f);
//			
//			}

}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	   if(hadc == &hadc1)
    {
  printf("o1");
     
    }
		   if(hadc == &hadc2)
    {
  printf("ok2");
     
    }
    if(hadc == &hadc3)
    {
  printf("ok3");
     
    }
}



