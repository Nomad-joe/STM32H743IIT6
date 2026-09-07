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
#include "./CMSIS/DSP/Include/arm_math.h"


//单次模式的dma必须先关闭再启动

#define IC_Len  2000
volatile uint32_t IC_CH1[IC_Len] ={0};
uint8_t IC_Finsh =0;              //DMA搬运结束标志位
uint32_t total_tick =0;           
double tick =0;
int main(void)
{  

//    sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
   	printf("ok\r\n");
	
	  MX_TIM2_Init();
    HAL_TIM_IC_Start_DMA(&htim2,TIM_CHANNEL_1,IC_CH1,IC_Len);

	
    while (1)
    {        
      if(IC_Finsh==1)
			{ IC_Finsh=0;total_tick = 0;
	
			  for(uint16_t i=0;i<IC_Len-1;i++)
				{
					total_tick +=IC_CH1[i+1]-IC_CH1[i];
				}
			  tick = (double)total_tick/1999.0f;
				HAL_TIM_IC_Start_DMA(&htim2,TIM_CHANNEL_1,IC_CH1,IC_Len);
			}
			printf("Freq = %.2lf Khz\r\n",240000/tick);

			delay_ms(1000);

			
    }
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
	//	HAL_TIM_IC_Stop_DMA(&htim2,TIM_CHANNEL_1);
		IC_Finsh=1;
	}

}




