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
#define FFT_LEN     4096
#define CH_Num       2
#define AD7606_SAMPLE  SAMPLE_CNT * CH_Num

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;

float FFT_input[2*FFT_LEN] = {0};
float FFT_input_2[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};//存放频谱
float FFT_mag_2[FFT_LEN] ={0};
uint32_t Index = 0; //mag数组中最大值的下标
float Finally_Index ;
float Finally_Freq;  //准确的频率
float Finally_Amp;   //准确的幅值
float Finally_Phase; //准确的相位
float Finally_Phase_2; //准确的相位
float FFT_Amp;
float FFT_Amp_2;
uint32_t	Index_2;



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
#if 0        // 打印结果
        for(uint16_t i = 0; i < SAMPLE_CNT; i++)
        {
            printf("%.2f \n",(float)AD7606B_CH1[i] * 10000.0f / 32768);
        }
				delay_ms(1000);    
#endif	
#if 0			
      long long ad_ave=0;
      for(uint16_t i=0;i<512;i++)
       {
			  ad_ave +=  AD7606B_CH1[i];
			 }
			 ad_ave = ad_ave*10000.0f /32768.0f;
			 printf("ad_ave =%.2f mv \r\n",(float)ad_ave/512.0f);
#endif 			 
#if 1			
				for(uint16_t i=0;i<FFT_LEN;i++)
			{

				AD7606B_CH1[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗
				AD7606B_CH2[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗		
				
       FFT_input[2*i] = AD7606B_CH1[i];
			 FFT_input[2*i+1] = 0;
			 FFT_input_2[2*i] = AD7606B_CH2[i];
			 FFT_input_2[2*i+1] = 0;
			}
		
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input_2, 0, 1);
    arm_cmplx_mag_f32(FFT_input_2, FFT_mag_2, FFT_LEN);
		arm_max_f32(FFT_mag_2,FFT_LEN/2,&FFT_Amp_2,&Index_2);
   		
   Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
	 Finally_Freq = Finally_Index*2.44140625;
	
	 printf("测量频率1：%.2f Hz\r\n",Finally_Freq);	
			
	 Finally_Index=((Index_2-1)* FFT_mag_2[Index_2-1] * FFT_mag_2[Index_2-1] + Index_2 * FFT_mag_2[Index_2] * FFT_mag_2[Index_2] + (Index_2+1)* FFT_mag_2[Index_2+1] * FFT_mag_2[Index_2+1]) /(FFT_mag_2[Index_2-1] * FFT_mag_2[Index_2-1]+FFT_mag_2[Index_2] * FFT_mag_2[Index_2]+FFT_mag_2[Index_2+1] * FFT_mag_2[Index_2+1]);
	 float Finally_Freq_2 = Finally_Index *2.44140625;
	 printf("测量频率2：%.2f kHz\r\n",Finally_Freq_2);	
			
			
			
   double real_1,real_2 =0;
	 double img_1,img_2  =0;
   double phase_1,phase_2 = 0;	
//   real_1 = FFT_input[2*Index];
//	 img_1  = FFT_input[2*Index+1];
//	 printf("phase = %.2lf 度\r\n",atan2f(img_1,real_1)*180.0f/PI);	
     phase_1 = (FFT_input[2*Index+1]) / FFT_input[2*Index];
// 	  phase_1 = 	atan2f(img_1,real_1)*180.0f/PI;	
//   real_2 = FFT_input_2[2*Index_2];
//	 img_2  = FFT_input_2[2*Index_2+1];
//	 printf("phase = %.2lf 度\r\n",atan2f(img_2,real_2)*180.0f/PI);	
     phase_2  = FFT_input_2[2*Index+1]  /  FFT_input_2[2*Index];
		 
		 float tan1_2 = (phase_1 - phase_2) / (1 + phase_1*phase_2);
//   phase_2 = atan2f(img_2,real_2)*180.0f/PI;
    float  phase1_2 = 180.f *atan(tan1_2)/PI;
 		printf("相位差：%.2f 度\r\n",phase1_2);	
			
			
			
#endif
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

