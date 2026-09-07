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
#include "./BSP/DAC/my_dac.h"
#define FFT_LEN  20000



uint32_t g_timeout;
uint32_t time;


uint8_t adc1_dma_complete_flag=0;
uint8_t adc1_dma_halfcomplete_flag=0;
uint16_t DMA_Buffer[FFT_LEN] = {0};


uint32_t DAC_sin[FFT_LEN] ={0};
uint32_t DAC_ramp[FFT_LEN] ={0};
uint32_t DAC_sqr[FFT_LEN] ={0};

uint32_t my_dac[FFT_LEN] ={0};

int main(void)
{  
    //sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(192, 5, 2, 4);
    delay_init(480);
    usart_init(1152000);
    led_init();
    mpu_memory_protection();
    sdram_init();
    key_init();
	
    MX_TIM3_Init();
    MX_TIM7_Init();

    MX_ADC1_Init();//pa3
    MX_DAC1_Init();//pa5
	

//		for(uint32_t i=0;i<FFT_LEN;i++)
//	{
//	  DAC_sin[i] =(uint16_t)(2047*sin(2*PI/FFT_LEN *i)+2048);
//     if(i<FFT_LEN/2) DAC_ramp[i] =8190/FFT_LEN * i;
//		else DAC_ramp[i] = 8050 - 8050/FFT_LEN *i;
//		if(i<FFT_LEN/2) DAC_sqr[i] =4095;
//		else DAC_sqr[i]=0;
//	}
	
	 
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);delay_ms(10);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_Buffer, FFT_LEN);
		HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_2,(uint32_t*)my_dac,FFT_LEN,DAC_ALIGN_12B_R);
	  HAL_TIM_Base_Start(&htim7);  
		HAL_TIM_Base_Start(&htim3);  
    while (1)
    { 
      
   if(adc1_dma_halfcomplete_flag==1){
	   adc1_dma_halfcomplete_flag =0;
//		for(uint16_t i=0;i<FFT_LEN/2;i++){
//		 my_dac[i] = DMA_Buffer[i];
//		}
		 for(uint16_t i=0;i<FFT_LEN/2;i++)
		 {
		   printf("%.2f\r\n",(float)DMA_Buffer[i]*3.3/4095.0);
		 }
		 
		 
	 }
	 
	   if(adc1_dma_complete_flag==1){
	   adc1_dma_complete_flag =0;
		 for(uint16_t i=FFT_LEN/2;i<FFT_LEN;i++)
		 {
		   printf("%.2f\r\n",(float)DMA_Buffer[i]*3.3/4095.0);
		 }
//			 		for(uint16_t i=FFT_LEN/2;i<FFT_LEN;i++){
//		      my_dac[i] = DMA_Buffer[i];
//		}
	 }	 
		
  }
}



void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    adc1_dma_halfcomplete_flag = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc1_dma_complete_flag = 1;
}

