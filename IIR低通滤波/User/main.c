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


#define DMA_Len     20000   // 正弦波点数，可自行修改
#define AMPLITUDE   8191.0 // 振幅，14位居中摆幅
#define OFFSET      8192.0 // 直流偏置，让正弦波中心在14位中点

uint8_t adc1_dma_halfcomplete_flag = 0;
uint8_t adc1_dma_complete_flag = 0;
uint16_t DMA_Buffer[DMA_Len] = {0};
uint8_t flag=0;
uint16_t Filtered_Buffer[DMA_Len];
// 14位DAC数据，uint16_t 足够，DMA/IO输出都兼容
uint16_t sine_buffer[DMA_Len];

void Generate_SineWave(void)
{
    for (int i = 0; i < DMA_Len; i++)
    {
        // 计算相位 0~2π
        double phase = 2.0 * PI * i / DMA_Len;
        // 正弦计算 + 偏置，范围：0 ~ 16383
        double val = sin(phase) * AMPLITUDE + OFFSET;
        
        // 14位DAC限幅：0 ~ 16383
        if(val > 16383.0) val = 16383.0;
        if(val < 0.0)     val = 0.0;
        
        // 四舍五入转整数
        uint16_t dac_code = (uint16_t)(val + 0.5);
        // 14位掩码，屏蔽高位，只保留 D0~D13
        dac_code &= 0x3FFF;
        
        sine_buffer[i] = dac_code;
    }
}



#define SAMPLE_FREQ  4000000.0f
#define CUTOFF_FREQ  10000.0f

// IIR滤波器变量
//float32_t b0;
//float32_t b1;
//float32_t b2;
//float32_t a1;
//float32_t a2;

float b0 = -0.4516893029f;
float b1 = 0.9037349224f;
float b2 = -0.4520174265f;
float a1 = -1.9926913977f;
float a2 = 0.9926975369f;
float32_t x_prev1 = 0, x_prev2 = 0;
float32_t y_prev1 = 0, y_prev2 = 0;

// 滤波后的数据缓冲区
uint16_t Filtered_Buffer[DMA_Len];

// ADC: 12位 0~4095 -> 0~3.3V
#define ADC_TO_VOLT(adc)  (((float32_t)(adc) / 4095.0f) * 3.3f)

// DAC: 14位 0~16383 -> -2.5V ~ +2.5V  
#define DAC_TO_VOLT(dac)  (((float32_t)(dac) / 16383.0f) * 5.0f - 2.5f)
#define VOLT_TO_DAC(v)    ((uint16_t)(((v) + 2.5f) / 5.0f * 16383.0f + 0.5f))

/**
 * @brief 初始化IIR滤波器
 */
void IIR_Init(void)
{
    float32_t wc = 2.0f * PI * CUTOFF_FREQ / SAMPLE_FREQ;
    float32_t Q = 0.7071f;
    float32_t cos_w = cosf(wc);
    float32_t sin_w = sinf(wc);
    float32_t alpha = sin_w / (2.0f * Q);
    
    b0 = (1.0f - cos_w) / 2.0f;
    b1 = 1.0f - cos_w;
    b2 = (1.0f - cos_w) / 2.0f;
    a1 = -2.0f * cos_w;
    a2 = 1.0f - alpha;
    
    float32_t a0 = 1.0f + alpha;
    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;
}

/**
 * @brief IIR滤波处理
 *        输入：ADC数字量(0~4095)
 *        输出：DAC数字量(0~16383)
 */
/**
 * @brief IIR滤波处理 - 修正版
 */
/**
 * @brief IIR滤波处理 - 确保状态连续
 */
void IIR_Filter_Process(uint16_t *src, uint16_t *dst, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        // ADC数字量 -> 电压 (0~3.3V)
        float32_t adc_voltage = ((float32_t)src[i] * 3.3f) / 4095.0f;
        
        // 映射到DAC范围 (-2.5V~+2.5V)
        float32_t dac_voltage = (adc_voltage / 3.3f) * 5.0f - 2.5f;
        
        // IIR滤波
        float32_t y = b0*dac_voltage + b1*x_prev1 + b2*x_prev2 - a1*y_prev1 - a2*y_prev2;
        
        // 更新状态
        x_prev2 = x_prev1;
        x_prev1 = dac_voltage;
        y_prev2 = y_prev1;
        y_prev1 = y;
        
        // 滤波后电压 -> DAC数字量(0~16383)
        dst[i] = (uint16_t)(((y + 2.5f) / 5.0f) * 16383.0f + 0.5f);
    }
}
int main(void)
{  
    //sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(1152000);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
	
	MX_GPIO_Init();   //初始化dac904所需所有的PC口
    Generate_SineWave();
    MX_TIM12_Init();      //4M pwm 驱动dac
	MX_TIM3_Init();       // 4M 驱动adc
	MX_DMA_Init();        //dac数据搬运dma 循环模式
	MX_ADC1_Init();       //PA3
	IIR_Init();	 
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//PH9 做clk
		 

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	delay_ms(20);
		 
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_Buffer, DMA_Len);
    HAL_TIM_Base_Start(&htim3);
		 
    
    
    while (1)
    {  
	if(adc1_dma_halfcomplete_flag==1){
		adc1_dma_halfcomplete_flag=0;
		IIR_Filter_Process(DMA_Buffer, Filtered_Buffer, DMA_Len/2); 
			 
		if(flag==0){ 
			flag =1;
			HAL_DMA_Start(&hdma_dma_generator0, 
              (uint32_t)Filtered_Buffer, 
              (uint32_t)&GPIOC->ODR, 
              DMA_Len);
		}
	}

	if(adc1_dma_complete_flag==1){
	    adc1_dma_complete_flag=0;
        IIR_Filter_Process(&DMA_Buffer[DMA_Len/2], &Filtered_Buffer[DMA_Len/2], DMA_Len/2);
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