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
uint32_t Index_2 =0; //二次谐波
uint32_t Index_3 =0; //三次
uint32_t Index_4 =0;
uint32_t Index_5 =0;


float Finally_Index ;
float Finally_Freq;
float Finally_Amp;
float Finally_Amp_2;
float Finally_Amp_3;
float Finally_Amp_4;
float Finally_Amp_5;

float FFT_Amp;
float DC ;//用于减去直流量

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

  
    while (1)
    {   HAL_TIM_Base_Start(&htim3);     
       HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC开始采集，DMA开启搬运
			delay_ms(1000);
			HAL_TIM_Base_Stop(&htim3);
			HAL_ADC_Stop_DMA(&hadc1);
			
       DC = 0;
			for(uint16_t i=0;i<FFT_LEN;i++){  //计算直流量
       DC += (float) DMA_Buffer[i] / FFT_LEN;
			}
			
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
//		arm_cfft_radix4_instance_f32 s;
//    arm_cfft_radix4_init_f32(&s, FFT_LEN, 0, 1);
//    arm_cfft_radix4_f32(&s, FFT_input);
//    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
			
  //  for(uint16_t i=0;i<FFT_LEN/2;i++){printf("%.3f\n",FFT_mag[i]);} 
			
	 Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
	 Finally_Freq = Finally_Index*244.140625;
	 Finally_Freq=Finally_Freq/1000;
			
   Finally_Amp = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1]);
   Finally_Amp /=1.25;

   Index_2 = round(2*Finally_Index);		
	 Index_3 = round(3*Finally_Index);
   Index_4 = round(4*Finally_Index);
   Index_5 = round(5*Finally_Index);
	 
   Finally_Amp_2 = sqrt(FFT_mag[Index_2-1] * FFT_mag[Index_2-1] + FFT_mag[Index_2] * FFT_mag[Index_2] + FFT_mag[Index_2+1] * FFT_mag[Index_2+1]) /1.21;
	 Finally_Amp_3 = sqrt(FFT_mag[Index_3-1] * FFT_mag[Index_3-1] + FFT_mag[Index_3] * FFT_mag[Index_3] + FFT_mag[Index_3+1] * FFT_mag[Index_3+1]) /1.22;
   Finally_Amp_4 = sqrt(FFT_mag[Index_4-1] * FFT_mag[Index_4-1] + FFT_mag[Index_4] * FFT_mag[Index_4] + FFT_mag[Index_4+1] * FFT_mag[Index_4+1]) /1.23;
   Finally_Amp_5 = sqrt(FFT_mag[Index_5-1] * FFT_mag[Index_5-1] + FFT_mag[Index_5] * FFT_mag[Index_5] + FFT_mag[Index_5+1] * FFT_mag[Index_5+1]) /1.25;	 
	 float THD = sqrt(Finally_Amp_2*Finally_Amp_2 + Finally_Amp_3*Finally_Amp_3 + Finally_Amp_4*Finally_Amp_4 + Finally_Amp_5*Finally_Amp_5) / Finally_Amp;
	 printf("THD = %.2f %%\r\n",THD*100);
	 printf("基波频率：%.4f kHz\r\n",Finally_Freq);
   printf("基波幅值: %.3f mv\r\n",Finally_Amp);
	 printf("二次幅值: %.3f mv\r\n",Finally_Amp_2);
	 printf("三次幅值: %.3f mv\r\n",Finally_Amp_3);
	 printf("四次幅值: %.3f mv\r\n",Finally_Amp_4);
	 printf("五次幅值: %.3f mv\r\n",Finally_Amp_5);	
	 printf("一次标幺 = %.3f\r\n",1.000);
	 printf("二次标幺 = %.3f\r\n",Finally_Amp_2/Finally_Amp);
	 printf("三次标幺 = %.3f\r\n",Finally_Amp_3/Finally_Amp);
	 printf("四次标幺 = %.3f\r\n",Finally_Amp_4/Finally_Amp);
	 printf("五次标幺 = %.3f\r\n",Finally_Amp_5/Finally_Amp);
	 
	 
  	 LED0_TOGGLE();delay_ms(250);
		}

		

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc ==&hadc1){

	}
}


