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
#include "./BSP/DMA/my_dma.h"	//驱动并行dac
#include "math.h"
#include "./BSP/AD9834/my_ad9834.h"//dds给激励
#include "./BSP/IIR/my_iir.h"


//#define DMA_Len     10000   // 正弦波点数，可自行修改
//#define AMPLITUDE   8191.0 // 振幅，14位居中摆幅
//#define OFFSET      8192.0 // 直流偏置，让正弦波中心在14位中点

//uint8_t adc1_dma_halfcomplete_flag = 0;
//uint8_t adc1_dma_complete_flag = 0;
//uint16_t DMA_Buffer[DMA_Len] = {0};
//uint8_t flag=0;
//uint16_t Filtered_Buffer[DMA_Len];


//#define SAMPLE_FREQ  4000000.0f


//// IIR滤波器变量
//float32_t b0, b1, b2, a1, a2;
//float32_t x_prev1 = 0, x_prev2 = 0;
//float32_t y_prev1 = 0, y_prev2 = 0;

//// 滤波后的数据缓冲区
//uint16_t Filtered_Buffer[DMA_Len];

//// ADC: 12位 0~4095 -> 0~3.3V
//#define ADC_TO_VOLT(adc)  (((float32_t)(adc) / 4095.0f) * 3.3f)

//// DAC: 14位 0~16383 -> -2.5V ~ +2.5V  
//#define DAC_TO_VOLT(dac)  (((float32_t)(dac) / 16383.0f) * 5.0f - 2.5f)
//#define VOLT_TO_DAC(v)    ((uint16_t)(((v) + 2.5f) / 5.0f * 16383.0f + 0.5f))


float PID_Set,err,err_add,dds_set;//PID参数
float Kp = 1 ;
float Ki = 10;
uint8_t pid_flag;//PID启动标志位

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
#define FFT_LEN  4096
uint32_t ADC2_Buffer[FFT_LEN];
 
float ADC_float [FFT_LEN] = {0};
float FFT_input[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};
uint32_t Index = 0; //mag数组中最大值的下标
float Finally_Amp;
float FFT_Amp;
uint8_t ADC3_flag=0;//adc2采样完成标志位 第四问
void RX_Change(void);
void Rx_Delete(void);


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
	AD9834_Init();
	MX_TIM4_Init();       //驱动adc3做pid 10K
	MX_GPIO_Init();       //初始化dac904所需所有的PC口
  MX_TIM12_Init();      //4M pwm 驱动dac
	MX_TIM3_Init();       // 4M 驱动adc
	MX_DMA_Init();        //dac数据搬运dma 循环模式
	
	MX_ADC1_Init();       //PA3  12bit
	MX_ADC2_Init();       //PB0 12bit
  MX_ADC3_Init();       //PF10
  HAL_TIM_Base_Start(&htim3);
  //HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//PH9 做clk
	
		 
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	delay_ms(20);

//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_Buffer, DMA_Len);
//  HAL_TIM_Base_Start(&htim3);

//   	 if(adc1_dma_halfcomplete_flag==1){
//		adc1_dma_halfcomplete_flag=0;
//		IIR_Filter_Process(DMA_Buffer, Filtered_Buffer, DMA_Len/2); 
//			 
//		if(flag==0){ 
//			flag =1;
//			HAL_DMA_Start(&hdma_dma_generator0, 
//              (uint32_t)Filtered_Buffer, 
//              (uint32_t)&GPIOC->ODR, 
//              DMA_Len); 
//    	if(adc1_dma_complete_flag==1){
//	    adc1_dma_complete_flag=0;
//        IIR_Filter_Process(&DMA_Buffer[DMA_Len/2], &Filtered_Buffer[DMA_Len/2], DMA_Len/2);
    pid_flag =1;
		AD9834_WaveSeting(1000,0,1,0);PID_Set = 1.2;AD9834_AmpSet(100);
		while (1)
    {  
			
			if(pid_flag==1){

     HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC2_Buffer, FFT_LEN);
	
			  while(ADC3_flag==0){};ADC3_flag=0;
				for(uint16_t i=0;i<FFT_LEN;i++)
			{
			  ADC_float[i] = (float)ADC2_Buffer[i]*3.3f/4095;
				ADC_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗
			  FFT_input[2*i] = ADC_float[i];
			  FFT_input[2*i+1] = 0;
			}
       arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_input, 0, 1);
       arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
			FFT_mag[0] =0;FFT_mag[1] =0;FFT_mag[2] =0;
			arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			printf("频率:%.2f hz\r\n",Index * 10000/4096.0f);
   Finally_Amp = sqrt(FFT_mag[Index-1] * FFT_mag[Index-1] + FFT_mag[Index] * FFT_mag[Index] + FFT_mag[Index+1] * FFT_mag[Index+1]);
   Finally_Amp /= 1.24832;
   printf("测量Vpp: %.3f v\r\n",Finally_Amp);
//		 err = 	Finally_Amp - PID_Set ;
//		 err_add +=err;
//			dds_set  = Kp * err + Ki *err_add;
//			AD9834_AmpSet(dds_set);
			
			}

	}

}


//注意的是：包头是d，包尾必须是f，否则会进入死循环

void RX_Change(void)//0：48 d：100 
{
  if(g_usart_rx_buf[0] == 100)//d:100 做包头，f：102做包尾，arr[1]以后用作模式切换
	{
    float num = 0;//蕴含信息转换后存放在num中
	  float point_ten = 10;//用于递进十分位百分位
		uint8_t i = 2;//数据信息从第2位开始
		
		uint8_t point = 0;//用于区分小数点前后
		while(g_usart_rx_buf[i] != 102)//不是包尾
		{
		  if(g_usart_rx_buf[i] == 46){point=1;i++;continue;}//该位是小数点
			if(point ==1){num += (g_usart_rx_buf[i]-48)/point_ten;point_ten*=10;i++;continue;}
		  else {num *=10; num +=g_usart_rx_buf[i]-48;i++;continue;}		
		}
		 if(g_usart_rx_buf[1] ==50) {AD9834_WaveSeting(num,0,1,0);AD9834_AmpSet(255);}//第二问
		else if(g_usart_rx_buf[1] ==51){AD9834_WaveSeting(1000,0,1,0);AD9834_AmpSet(45); }//第三问
		else if(g_usart_rx_buf[1] ==52){PID_Set = num;pid_flag=1;}
	}
Rx_Delete();
}
void Rx_Delete(void)
{
  for(uint8_t i=0;i<50;i++)g_usart_rx_buf[i] = 0;
  g_usart_rx_sta =0;
}




void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    //adc1_dma_halfcomplete_flag = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{   if(hadc->Instance == ADC1){ /*adc1_dma_complete_flag = 1;*/}
	  else if(hadc->Instance == ADC3){  ADC3_flag =1 ;}
}
