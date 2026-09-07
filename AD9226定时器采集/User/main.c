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
#include "./BSP/DMA/my_dma.h"	


#include "math.h"

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
#define FFT_LEN  4096 
uint16_t DMA_Buffer[FFT_LEN+20] = {0};
float AD9226_Voltage[FFT_LEN];
float FFT_input[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};
uint32_t Index = 0; //mag数组中最大值的下标
uint32_t Index2 =0;
uint32_t Index3 =0;
float Finally_Index ;
float Finally_Freq;
float Finally_Amp;
float FFT_Amp;


void AD9226_ConvertToVoltage(uint16_t *adc_buf, float *vol_buf, uint32_t len);
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
	
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	  MX_GPIO_Init();   //初始化dac904所需所有的PI口 和 PH9 clk


    while (1)
    {    
		 AD9226_Read_Data(0);
	   AD9226_ConvertToVoltage(AD9226_Data,AD9226_Voltage,FFT_LEN+20);
#if 1		 
		 for(uint16_t i =20;i<FFT_LEN+20;i++)
		 {
		   printf("%.2f\r\n",AD9226_Voltage[i]*1000);
		 }
#endif
#if 0	 
		   for(uint16_t i =0;i<FFT_LEN;i++)//转变为复信号
			{
			 FFT_input[2*i] = AD9226_Voltage[i+10];
			 FFT_input[2*i+1] = 0;
			}

      arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
      arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
			FFT_mag[0] = 0;FFT_mag[1] = 0;FFT_mag[2] = 0;
			arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			
     


	 Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
	 Finally_Freq = Finally_Index*244.140625*2;
	 Finally_Freq=Finally_Freq/1000;
			
   Finally_Amp = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1]);
   Finally_Amp  /=1.92;
	 printf("测量频率：%.4f KHz\r\n",Finally_Freq);
   printf("测量幅值: %.3f mv\r\n",Finally_Amp);
	 
#endif	 
	 delay_ms(500);
		}
 }

void AD9226_ConvertToVoltage(uint16_t *adc_buf, float *vol_buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        // 只提取低12位有效数据
        uint16_t adc_value = adc_buf[i] & 0x0FFF;
        
        // 可选：检查过压标志（如果AD9226输出13位，第12位是OTR）
        // uint8_t otr = (adc_buf[i] >> 12) & 0x01;
        
        // 转换公式：电压 = (adc / 4095) * 10 - 5
        vol_buf[i] = ((float)adc_value / 4095.0f) * 10.0f - 5.0f;
    }
}



