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
/*  
 D15接地是spi模式
 D7接E5
 CS接I9
 BUSY接I8
 RST接E4
 RD(clk)接E2
 CNT接I6
 RAE接高电平
 GND接GND




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
uint16_t temp;//临时变量
volatile  static	int16_t AD7606B_Data[SAMPLE_CNT] = {0};
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
	  TIM8_CH2_PI6_PWM_Init();//产生驱动信号 	CNT
    while (1)
    {   printf("ok");
        AD7606B_ADC(SAMPLE_CNT);
			printf("end");
 //       SCB_InvalidateDCache_by_Addr((uint32_t*)AD7606B_Data, sizeof(AD7606B_Data));
        // 打印结果
        for(uint16_t i = 0; i < SAMPLE_CNT; i++)
        {
            printf("%.2f \n",(float)AD7606B_Data[i] * 10000.f / 32768);
        }
       
        delay_ms(1000);
			}
}
void AD7606B_ADC(uint16_t num)
{
   
    sample_index = 0;
    sample_finish_flag = 0;

    AD7606B_CS_H();
    delay_us(2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    
    while(sample_finish_flag == 0); // 等待完成
    
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    AD7606B_CS_H();
}

void EXTI9_5_IRQHandler(void)
{    
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
    {  
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
        
        if(sample_index < SAMPLE_CNT)
        {
            AD7606B_CS_L();
            HAL_SPI_TransmitReceive(&hspi4, (uint8_t *)&TxData, (uint8_t *)&temp, 1, 100);
            AD7606B_CS_H();
            
            AD7606B_Data[sample_index++] = (int16_t)temp;
            
            
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

