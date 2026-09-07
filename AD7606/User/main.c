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
#include "./BSP/AD7606/my_ad7606.h"
#include "./CMSIS/DSP/Include/arm_math.h"

	uint8_t i;
	uint8_t buf[40];
static	int16_t AD7606B_Data[8] = {0};
	
int main(void)
{  
    //sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
  
    AD7606B_Init();

    while (1)
    {  
		AD7606B_Start_Convst();
		delay_us(1);
		while((AD7606B_BUSY == GPIO_PIN_SET))	/* 读取BUSY的状态，为低电平表示电平转换结束，可以读取数据 */
		{
			__NOP();
		}
		AD7606B_Read_AD_Data(AD7606B_Data);		/* 读取数据放至数组AD7606B_Data[] */
		for(i=0;i<1;i++)
		{
			sprintf((char *)buf,"CH%1d:%7.2fmV  0x%04X     ",i+1, (float)((float)AD7606B_Data[i]*AD7606B_Range/32768), (uint16_t)AD7606B_Data[i]);
//		printf("CH1 = %d\r\n",AD7606B_Data[i]);
			printf("%s",buf);
		}
		printf("\r\n");
		delay_ms(2000);
			
			
			
		}
}





