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

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
 	
#define FFT_LEN  4096
uint32_t DMA_Buffer[FFT_LEN] = {0};

float ADC1_float [FFT_LEN] = {0};
float ADC2_float [FFT_LEN] = {0};
float FFT_input[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN];
uint32_t Index = 0; //mag数组中最大值的下标
float FFT_Amp;
float Finally_Index ;
float Finally_Freq;  //准确的频率
float Finally_Amp;   //准确的幅值



void Process_Data(uint32_t* src_buf, uint16_t num_pairs) ;
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
 
	
	MX_TIM3_Init();

	MX_ADC1_Init();//PA2
	MX_ADC2_Init();
	
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_FACTOR_LINEARITY_REGOFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_FACTOR_LINEARITY_REGOFFSET, ADC_SINGLE_ENDED);
	
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t* )DMA_Buffer,FFT_LEN);	
  HAL_ADC_Start(&hadc2);

  
    while (1)
    {  
  		 
	     HAL_TIM_Base_Start(&htim3);
	     delay_ms(1000);
			Process_Data(DMA_Buffer,FFT_LEN);
			HAL_ADC_Stop_DMA(&hadc1);
		  HAL_TIM_Base_Stop(&htim3);
			
			for(uint16_t i=0;i<FFT_LEN;i++)
			{
				ADC1_float[i] = (float)DMA_Buffer[i]*3.3f/65536;
				printf("%.2f\r\n",ADC1_float[i]);
				ADC1_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗
        FFT_input[2*i] = ADC1_float[i];
				FFT_input[2*i+1] = 0;
			}
		while(1){};
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
		FFT_mag[0]=0;FFT_mag[1]=0;FFT_mag[2]=0;
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
		
	 Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
	 Finally_Freq = Finally_Index*366.2109375*2;
	 Finally_Freq=Finally_Freq/1000;
	
	 printf("测量频率：%.2f kHz\r\n",Finally_Freq);




}

}

void Process_Data(uint32_t* src_buf, uint16_t num_pairs) {
    // 1. 分离交织数据
    for (uint16_t i = 0; i < num_pairs; i++) {
        uint32_t raw = src_buf[i];
        ADC1_float[i] = (uint16_t)(raw & 0x0FFF); // 假设12位数据在低位
        ADC2_float[i] = (uint16_t)((raw >> 16) & 0x0FFF);
    }

}



