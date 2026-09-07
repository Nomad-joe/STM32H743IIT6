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
#include "./BSP/DAC/my_dac.h"
#include "./CMSIS/DSP/Include/arm_math.h"

uint32_t g_timeout;
uint32_t time;

#define DAC_Len 1024
uint32_t DAC_sin[DAC_Len] ={0};
uint32_t DAC_ramp[DAC_Len] ={0};
uint32_t DAC_sqr[DAC_Len] ={0};
#define duty  0.5
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

	for(uint32_t i=0;i<DAC_Len;i++)
	{
	  DAC_sin[i] =(uint16_t)(2047*sin(2*PI/DAC_Len *i)+2048);
     if(i<DAC_Len/2) DAC_ramp[i] =8190/DAC_Len * i;
		else DAC_ramp[i] = 8050 - 8050/DAC_Len *i;
		if(i<DAC_Len/2) DAC_sqr[i] =4095;
		else DAC_sqr[i]=0;
		printf("DAC_sin[%d] = %d\r\n",i,DAC_sin[i]);
	}
	
    MX_DAC1_Init();//DAC初始化
    MX_TIM7_Init();//DAC驱动时钟初始化
	  HAL_TIM_Base_Start(&htim7);
	  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_2,(uint32_t*)DAC_ramp,DAC_Len,DAC_ALIGN_12B_R);
    while (1)
    { 

		}
		
		
}






