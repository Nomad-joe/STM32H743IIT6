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

float adc_float;
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
    
    MX_ADC1_Init();
		HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
  while (1)
  {
    delay_ms(500);
    HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,10); 
    uint16_t adc_vol =  HAL_ADC_GetValue(&hadc1);
		 adc_float = (float)adc_vol*3300.0f/65535.0f;
    printf("adc_float =%.2f\r\n",adc_float);
  }
}





