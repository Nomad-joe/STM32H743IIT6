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

void show_work(void);
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
    MX_USART2_UART_Init(115200);
	
	
    MX_TIM5_Init();//pa0 输出pwm
	  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	
	  delay_ms(3000);
	
	
	  __HAL_TIM_SET_AUTORELOAD(&htim5, 180); 
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 180/2);
    while (1)
    {  
			show_work();
			delay_ms(100);
		}
}
void show_work(void)
{
if(g_usart_rx_sta_2 != 0)
{ 
 for(uint8_t i=0;i<g_usart_rx_sta_2;i++)
	{
	   printf("%c",g_usart_rx_buf_2[i]);
	}
	g_usart_rx_sta_2=0;
   printf("\r\n");
}

}




