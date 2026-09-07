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

#define FFT_LEN  1024

uint32_t g_timeout;
uint32_t time;
uint8_t adc1_dma_complete_flag=0;//存放DMA搬运完成标志位
uint8_t adc2_dma_complete_flag=0;

uint16_t DMA_Buffer[FFT_LEN] = {0};//存放dma搬运数据
uint16_t DMA_Buffer_2[FFT_LEN] = {0};

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;
 	
float ADC_float [FFT_LEN] = {0};//转换的真实电压
float ADC_float_2 [FFT_LEN] = {0};

float FFT_input[2*FFT_LEN] = {0};//FFT输入数组
float FFT_input_2[2*FFT_LEN] = {0};

float FFT_mag[FFT_LEN] ={0};//存放频谱
float FFT_mag_2[FFT_LEN] ={0};

uint32_t Index = 0; //mag数组中最大值的下标

float Freq_A = 0;//A信号频率信息
float Freq_Set_B = 0;//B信号的设定频率信息
float FFT_Amp;//幅值信息




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
 
	timx_int_init(65535, 240 - 1);          /* 1Mhz计数频率,设置自动重载值为65536 */


	MX_TIM3_Init();
  PeriphCommonClock_Config();
	MX_ADC1_Init();//PA0
	MX_ADC2_Init();//PA7
 
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
    while (1)
    { 
	     TIM6->CNT = 0;                          /* 重设定时器TIM6的计数器值 */
       g_timeout = 0;	
			HAL_TIM_Base_Start(&htim3);
			HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC1开始采集，DMA开启搬运
			HAL_ADC_Start_DMA(&hadc2,(uint32_t*)DMA_Buffer_2,FFT_LEN);//ADC2开始采集，DMA开启搬运
			
      
      while(adc1_dma_complete_flag==0 | adc2_dma_complete_flag==0){}//等待采集完成    
				adc1_dma_complete_flag=0;adc2_dma_complete_flag=0;

		  HAL_TIM_Base_Stop(&htim3);
			HAL_ADC_Stop_DMA(&hadc1);
			HAL_ADC_Stop_DMA(&hadc2);
			for(uint16_t i=0;i<FFT_LEN;i++)//一个循环里完成转换实际电压 加窗 转变复信号
			{
			  ADC_float[i] = (float)DMA_Buffer[i]*3.3f/65536;
				ADC_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗
			  FFT_input[2*i] = ADC_float[i];
				FFT_input[2*i+1] = 0;
				
			  ADC_float_2[i] = (float)DMA_Buffer_2[i]*3.3f/65536;
				ADC_float_2[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗				
        FFT_input_2[2*i] = ADC_float_2[i];
				FFT_input_2[2*i+1] = 0;
			}
			
			
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_input, 0, 1);//FFT	得到复数谱
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);         //得到幅度谱
		FFT_mag[0] = 0; FFT_mag[1] = 0;			//去除直流干扰
    
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);         //找出最大索引
			
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_input_2, 0, 1);
//    arm_cmplx_mag_f32(FFT_input_2, FFT_mag_2, FFT_LEN);
//		arm_max_f32(FFT_mag_2,FFT_LEN/2,&FFT_Amp_2,&Index_2);

   	Freq_A = Index *200.0f;
    printf("测量频率：%.2f Hz\r\n",Freq_A);
			
   float real_1,real_2 =0;
	 float img_1,img_2  =0;
   
   real_1 = FFT_input[2*Index];
	 img_1  = FFT_input[2*Index+1];
	 float phase_1 = 	atan2f(img_1,real_1)*180.0f/PI;	//得到A信号相位
			
   real_2 = FFT_input_2[2*Index];
	 img_2  = FFT_input_2[2*Index+1];
			
   float phase_2 = atan2f(img_2,real_2)*180.0f/PI;       //得到B信号相位
	 float diff = 	phase_2-phase_1;
	 if(diff>180)diff -=360;
	 if(diff<-180)  diff +=360;
	 
	 printf("相位差：%.2f 度\r\n",phase_2-phase_1);	
	 
    time = TIM6->CNT + (uint32_t)g_timeout * 65536;             /* 计算所用时间 */
    printf("%0.3fms\r\n", (float)time / 1000);








}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        adc1_dma_complete_flag = 1;  // ADC1 采集完成，置标志位
    }
    
    if(hadc->Instance == ADC2)
    {
        adc2_dma_complete_flag = 1;  // ADC2 采集完成，置标志位
    }
}



