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
#include <inttypes.h>
uint32_t g_timeout;
uint32_t time;
uint8_t Mode_Now = 0;
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
 	
#define FFT_LEN  4096 
uint16_t DMA_Buffer[FFT_LEN] = {0};
uint16_t DMA_Buffer_2[FFT_LEN] = {0};

float ADC_float [FFT_LEN] = {0};
float ADC_float_2 [FFT_LEN] = {0};

float FFT_input[2*FFT_LEN] = {0};
float FFT_input_2[2*FFT_LEN] = {0};

float FFT_mag[FFT_LEN] ={0};//存放频谱
float FFT_mag_2[FFT_LEN] ={0};

uint32_t Index = 0; //mag数组中最大值的下标
uint32_t Index_2 =0;

float Finally_Index ;
float Finally_Freq;  //准确的频率
float Finally_Amp;   //准确的幅值


float FFT_Amp;
float FFT_Amp_2;


#define CAPTURE_BUF_SIZE 100  // 你可以改大小
volatile uint32_t cap_buf_ch2[CAPTURE_BUF_SIZE];  // TIM2_CH2 DMA 目标数组
volatile uint32_t cap_buf_ch3[CAPTURE_BUF_SIZE];  // TIM2_CH3 DMA 目标数组
void Select_Mode(uint8_t Mode);
int main(void)
{  
 //   sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
    MX_USART2_UART_Init(115200);
	  relay_init();
	
	MX_TIM2_Init();//PA1 PA2 DMA测频  
	MX_TIM3_Init();
	
  PeriphCommonClock_Config();
	MX_ADC1_Init();//PA0  uc
	MX_ADC2_Init();//PA7  ur
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
  delay_ms(20);

//   RCC->AHB4ENR |= (1 << 4);   // 使能 GPIOE 时钟
//    
//    // 2. 配置 PE2 为输出模式
//    GPIOE->MODER &= ~(3 << 4);   // 清除 MODER2 位
//    GPIOE->MODER |= (1 << 4);    // 设为输出模式
    GPIOE->BSRR = GPIO_PIN_3;
		GPIOE->BSRR = (uint32_t)GPIO_PIN_3 << 16;
//		GPIOE->BSRR = GPIO_PIN_3;
    while (1)
    {
#if 1
			HAL_TIM_Base_Start(&htim3);
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC1开始采集，DMA开启搬运
			HAL_ADC_Start_DMA(&hadc2,(uint32_t*)DMA_Buffer_2,FFT_LEN);//ADC2开始采集，DMA开启搬运
		 	delay_ms(1000);
			HAL_ADC_Stop_DMA(&hadc1);
			HAL_ADC_Stop_DMA(&hadc2);
		  HAL_TIM_Base_Stop(&htim3);
			delay_ms(10);

      for(uint16_t i=0;i<FFT_LEN;i++)
			{
			  ADC_float[i] = (float)DMA_Buffer[i]*3.3f/65536;
				ADC_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗
				
			  ADC_float_2[i] = (float)DMA_Buffer_2[i]*3.3f/65536;
				ADC_float_2[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗				
			}
			
   for(uint16_t i=0;i<FFT_LEN;i++)//转变为复信号
			{
			 FFT_input[2*i] = ADC_float[i];
			 FFT_input[2*i+1] = 0;
				
			 FFT_input_2[2*i] = ADC_float_2[i];
			 FFT_input_2[2*i+1] = 0;
			}
			
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
		FFT_mag[0]=0;FFT_mag[1]=0;FFT_mag[2]=0;
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input_2, 0, 1);
    arm_cmplx_mag_f32(FFT_input_2, FFT_mag_2, FFT_LEN);
//		arm_max_f32(FFT_mag_2,FFT_LEN/2,&FFT_Amp_2,&Index_2);
   		

	 Finally_Index=((Index-1)* FFT_mag[Index-1] * FFT_mag[Index-1] + Index * FFT_mag[Index] * FFT_mag[Index] + (Index+1)* FFT_mag[Index+1] * FFT_mag[Index+1]) /(FFT_mag[Index-1] * FFT_mag[Index-1]+FFT_mag[Index] * FFT_mag[Index]+FFT_mag[Index+1] * FFT_mag[Index+1]);
	 Finally_Freq = Finally_Index*58.59375;
	 Finally_Freq=Finally_Freq/1000;
			
   Finally_Amp = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1]);
   Finally_Amp *= 2*1.708/1.873;
			
   FFT_Amp_2 = sqrt(FFT_mag_2[Index-1] * FFT_mag_2[Index-1] + FFT_mag_2[Index] * FFT_mag_2[Index] + FFT_mag_2[Index+1] * FFT_mag_2[Index+1]);
   FFT_Amp_2 *= 2*1.708/1.851;			

   float C = 1.0f/(2*3.1415*2000*1000)*FFT_Amp_2 /Finally_Amp *10;
	 
	 printf("测量频率：%.2f kHz\r\n",Finally_Freq);
	 if(Finally_Freq<3){
   printf("测量幅值1: %.3f mv\r\n",Finally_Amp);
   printf("测量幅值2: %.3f mv\r\n",FFT_Amp_2);
	 printf("电容值   : %.8f nF\r\n",C*100000000);
	 }
	 else
	 {
	 printf("测量幅值1: %.3f mv\r\n",Finally_Amp/1.079);
   printf("测量幅值2: %.3f mv\r\n",FFT_Amp_2/1.135);
	 float L = 100.0f/(2*100000*3.1415)*(Finally_Amp/1.079/(FFT_Amp_2/1.135));
	 printf("电感值   : %.8f μH\r\n",L*1000000);
	 }
	 
#endif 

#if  1

    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, cap_buf_ch2, CAPTURE_BUF_SIZE);
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, cap_buf_ch3, CAPTURE_BUF_SIZE);
	  delay_ms(100);
		int64_t phase_tick = 0;
    for(uint8_t i=0; i<100;i++)
		{
		  phase_tick += (int32_t) (cap_buf_ch3[i] - cap_buf_ch2[i]);
		}
		phase_tick /= 100.0f;
		printf("phase_tick : %" PRId64 " \r\n",phase_tick);
		uint32_t T_tick = (float) (cap_buf_ch3[1] - cap_buf_ch3[0]);
		
		printf("T_tick : %d \r\n",T_tick);
    float phase = (float)phase_tick*360 / T_tick ;
    phase -=180;
		phase -=0.29;
    while(phase>180)phase-=360;
		while(phase<-180)phase+=360;
		
		printf("相位差 : %.2f \r\n",phase);
		phase +=90;
		float tan_phase = tanf(phase * 0.0174532925f);
		printf("D值：%.4f\r\n",tan_phase);
#endif 
}

}
void Select_Mode(uint8_t Mode)
{
	if(Mode_Now!=Mode)
	{Mode_Now = Mode;
	  GPIOE->BSRR = (uint32_t)GPIO_PIN_2 << 16;
	  GPIOE->BSRR = (uint32_t)GPIO_PIN_3 << 16;
		GPIOE->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		GPIOE->BSRR = (uint32_t)GPIO_PIN_5 << 16;
		
		if(Mode_Now==1)	        {GPIOE->BSRR = GPIO_PIN_2;}
	  else if(Mode_Now==2)		{GPIOE->BSRR = GPIO_PIN_3;}
		else if(Mode_Now==3)		{GPIOE->BSRR = GPIO_PIN_4;}
		else if(Mode_Now==4)		{GPIOE->BSRR = GPIO_PIN_5;}
   }
}


