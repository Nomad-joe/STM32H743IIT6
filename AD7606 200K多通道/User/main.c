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

#define SAMPLE_CNT  4096 
#define CH_Num       2
#define AD7606_SAMPLE  SAMPLE_CNT * CH_Num


uint16_t AD7606_temp[AD7606_SAMPLE];
uint16_t CH1_temp[SAMPLE_CNT];
uint16_t CH2_temp[SAMPLE_CNT];
uint16_t buffer[2];  // 32位缓冲区
int16_t AD7606B_CH1[SAMPLE_CNT] = {0};
int16_t AD7606B_CH2[SAMPLE_CNT] = {0};
uint16_t TxData[8] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};//垃圾数据
uint16_t 	sample_index=0;
uint8_t   sample_finish_flag=0;
void AD7606B_ADC(uint16_t num);
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
    PI8_EXTI_Init();//BUSY
	  TIM8_CH2_PI6_PWM_Init();//产生驱动信号

    while (1)
    {  
        AD7606B_ADC(SAMPLE_CNT);//给予SAMPLE_CNT个脉冲
        // 打印结果
        for(uint16_t i = 0; i < SAMPLE_CNT; i++)
        {
            printf("%.2f \n",(float)AD7606B_CH2[i] * 10000.f / 32768);
        }
        
        printf("\r\n");
			

        delay_ms(2000);
			}
}
void AD7606B_ADC(uint16_t num)
{
   
    sample_index = 0;
    sample_finish_flag = 0;

    
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
     
    while(sample_finish_flag == 0); // 等待完成
    
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);

	
	 for(uint16_t i=0;i<SAMPLE_CNT;i++)
	{
	  AD7606B_CH1[i] = (int16_t)CH1_temp[i];
		AD7606B_CH2[i] = (int16_t)CH2_temp[i];
	}
	
}

void EXTI9_5_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
    {    AD7606B_CS_L();	
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

        if(sample_index < SAMPLE_CNT)
        {
//					HAL_SPI_TransmitReceive(&hspi4, (uint8_t *)&TxData, (uint8_t *)&CH1_temp[sample_index],2, 20);
     
        HAL_SPI_TransmitReceive(&hspi4, (uint8_t *)&TxData, (uint8_t *)&buffer, 2, 20);
         CH1_temp[sample_index] = (uint16_t)buffer[0];               
         CH2_temp[sample_index] = (uint16_t)buffer[1];

          sample_index +=1;    

					AD7606B_CS_H();
            if(sample_index >= SAMPLE_CNT)
            {   
                sample_finish_flag = 1;
            }
        }
        else
        {
            sample_finish_flag = 1;
        }
    }
}

