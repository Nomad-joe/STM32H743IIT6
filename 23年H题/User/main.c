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

#include "./BSP/AD9834/my_ad9834.h"
#include "./BSP/AD9834_2/my_ad9834_2.h"
#include "./CMSIS/DSP/Include/arm_math.h"

#define FFT_LEN  1024

uint8_t Wave_A_Pre =1;//当前波形检测值 1为正弦  0为三角波
uint8_t Wave_A =1;
uint8_t Wave_B =1;
uint8_t Wave_B_Pre =1;


float target_phase = 180.0f;     // 目标相位差
float target_phase_B = 180.0f;     // 目标相位差
float target_phase_A_B = 180;        //拓展部分AB信号相位差
uint32_t g_timeout;
uint32_t time;
uint8_t adc1_dma_complete_flag=0;//存放DMA搬运完成标志位
uint8_t adc2_dma_complete_flag=0;
uint8_t adc3_dma_complete_flag=0;

uint16_t DMA_Buffer[FFT_LEN] = {0};//存放dma搬运数据
uint16_t DMA_Buffer_2[FFT_LEN] = {0};
uint16_t dma_buffer_sram4[FFT_LEN] __attribute__((section(".ARM.__at_0x38000000")));

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;
 	
float ADC_float [FFT_LEN] = {0};//转换的C信号真实电压
float ADC_float_2 [FFT_LEN] = {0};//转换的A'信号真实电压
float ADC_float_3 [FFT_LEN] = {0};//转换的B'信号真实电压


float FFT_input[2*FFT_LEN] = {0};//FFT输入数组
float FFT_input_2[2*FFT_LEN] = {0};
float FFT_input_3[2*FFT_LEN] = {0};

float FFT_mag[FFT_LEN] ={0};//存放C频谱
float FFT_mag_2[FFT_LEN] ={0};//DDS A'信号频谱
float FFT_mag_3[FFT_LEN] ={0};//DDS B'信号频谱

uint32_t Index = 0; //mag数组中第一个极大值下标
uint32_t Index_2 =0;//第二大，B信号频率

float Freq_A = 0;//A信号频率信息
float Freq_B = 0;//B信号频率信息
float Freq_Set_A = 0;//A信号的设定频率信息
float Freq_Set_B = 0;//B信号的设定频率信息

float FFT_Amp_A;//幅值信息
float FFT_Amp_B;//幅值信息

uint8_t Lock_Flag =0; //锁相开始标志位


float Pre_Phase  = 0; //当前相位差


float Pre_Phase_B  = 0; //当前相位差


float Phase_Fix = 0;
float Phase_Fix_B=0;

uint8_t Counter =0;//打印频率，每100次调整打印一次
uint8_t flag =0;   //倍数频率锁相(只有最后一问成倍数时，此标志位为1)

void RX_Change(void);//修改设定值
void Rx_Delete(void);//清空串口缓冲数组

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
    AD9834_Init();                          /* 初始化DDS   AD9833 */
	  MX_TIM3_Init();
	
  PeriphCommonClock_Config();
	MX_ADC1_Init();//PA0
	MX_ADC2_Init();//PA7
  MX_ADC3_Init();//PC3
	
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC1校正函数
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC2校正函数
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);//ADC3校正函数
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC1开始采集，DMA开启搬运
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)DMA_Buffer_2,FFT_LEN);//ADC2开始采集，DMA开启搬运
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)dma_buffer_sram4,FFT_LEN);//ADC3开始采集，DMA开启搬运
  
	
    while (1)
    { 
      HAL_TIM_Base_Start(&htim3);
      while(adc1_dma_complete_flag==0 || adc2_dma_complete_flag==0 ||adc3_dma_complete_flag==0){}//等待采集完成    
				adc1_dma_complete_flag=0;adc2_dma_complete_flag=0;adc3_dma_complete_flag =0;
        
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
				
				ADC_float_3[i] = (float)dma_buffer_sram4[i]*3.3f/65536;
				ADC_float_3[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗				
        FFT_input_3[2*i] = ADC_float_3[i];
				FFT_input_3[2*i+1] = 0;
			}
		
			
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_input, 0, 1); //C 信号 FFT	得到复数谱
    arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);          //得到幅度谱
		FFT_mag[0] = 0; FFT_mag[1] = 0;			                    //去除直流干扰
			
    arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp_A,&Index);      //找出幅值最大的信号
    FFT_mag[Index] =0;                                     //该信号的幅值已经存放入FFT_Amp_A，频率在Index里包含。清零来寻找第二大的信号
			
		arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp_B,&Index_2);   //找到幅值第二大的信号
			
		if(Index>Index_2){  //A 信号的频率比B信号小，把A信号的信息存入Index和FFT_Amp_A，B信号的信息存入Index_2和FFT_Amp_B
			Index += Index_2; Index_2 = Index - Index_2;  Index = Index -  Index_2;
		  FFT_Amp_A +=FFT_Amp_B;FFT_Amp_B =FFT_Amp_A-FFT_Amp_B;FFT_Amp_A = FFT_Amp_A-FFT_Amp_B;
		                     }
												 
   /*  对AB信号波形分析  */											 
		if(FFT_Amp_A>165)Wave_A_Pre=1;  //如果幅值>165,A就是正弦波 反之三角
	 else  Wave_A_Pre=0;
	 if(FFT_Amp_B>165)Wave_B_Pre = 1; //如果幅值>165,B就是正弦波 反之三角
	 else Wave_B_Pre=0;				 
												 
												 
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_input_2, 0, 1);//DDS A' 信号FFT
    arm_cmplx_mag_f32(FFT_input_2, FFT_mag_2, FFT_LEN);
		

		arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_input_3, 0, 1);//DDS B'信号 FFT	
		arm_cmplx_mag_f32(FFT_input_3, FFT_mag_3, FFT_LEN);

   	Freq_A = Index *200.0f;     //得到C信号中A信号的频率
		Freq_B = Index_2 *200.0f;   //得到C信号中B信号的频率

    /*针对频率和波形变化时的改动*/				
	if( fabs(Freq_A-Freq_Set_A) > 2 ||Wave_A_Pre !=Wave_A ){ //频率变化>2hz或波形变化时刷新DDS频率或波形
		  Freq_Set_A  = Freq_A;
		  Wave_A = Wave_A_Pre;
		  AD9834_WaveSeting(Freq_Set_A,0,Wave_A,0);  //DDS设置新的频率和波形
		  Lock_Flag =0;
	}		
  if( fabs(Freq_B-Freq_Set_B) > 2 ||Wave_B_Pre !=Wave_B){
		  Freq_Set_B  = Freq_B;
		  Wave_B  = Wave_B_Pre;
		  AD9834_WaveSeting_2(Freq_Set_B,0,Wave_B,0);
		  Lock_Flag =0;
		}	
	
	else {Lock_Flag =1;}	
		
	if(Lock_Flag ==1)	{
		
   float real_1, img_1 =0;//C 信号中A信号相位
	 float real_2, img_2  =0;//DDS 中A信号相位
	
   real_1 = FFT_input[2*Index];
	 img_1  = FFT_input[2*Index+1];
	 float phase_1 = 	atan2f(img_1,real_1)* 180.0f / PI;	//得到A信号相位
			
   real_2 = FFT_input_2[2*Index];
	 img_2  = FFT_input_2[2*Index+1];
			
   float phase_2 =  atan2f(img_2,real_2)* 180.0f / PI;       //得到DDS_A信号相位
      Pre_Phase     =  phase_2-phase_1;
   while(Pre_Phase>=180)Pre_Phase -=360;
	 while(Pre_Phase<=-180)Pre_Phase+=360;	                   //得到当前相位差
	  


     Phase_Fix += target_phase - Pre_Phase;       //相位误差量 Pre_Phase - target_phase;
		 while(Phase_Fix>360)Phase_Fix-=360;
		 while(Phase_Fix<0)Phase_Fix+=360;

		  AD9834_Set_Phase(0,Phase_Fix);   


   float real_3, img_3 =0;//C 信号中B信号相位
	 float real_4, img_4 =0;//DDS 信号中B信号相位
   real_3 = FFT_input[2*Index_2];
	 img_3  = FFT_input[2*Index_2+1];
	 
	 float phase_3 = 	atan2f(img_3,real_3)* 180.0f / PI;	//得到B信号相位
			
   real_4 = FFT_input_3[2*Index_2];
	 img_4  = FFT_input_3[2*Index_2+1];
	
   float phase_4 =  atan2f(img_4,real_4)* 180.0f / PI;       //得到DDS_B信号相位
      Pre_Phase_B     =  phase_4-phase_3;
   while(Pre_Phase_B>=180)Pre_Phase_B -=360;
	 while(Pre_Phase_B<=-180)Pre_Phase_B+=360;	                   //得到当前相位差
   
	 
	           
	float Phase_A_B = phase_4 - (Freq_Set_B/Freq_Set_A)*phase_2;//求出A~ B~信号相位差
	Phase_A_B -= (Freq_Set_B/Freq_Set_A)*(-90.0f)+450;
	while(Phase_A_B>=360)Phase_A_B-=360;
	while(Phase_A_B<=0)Phase_A_B+=360;
//  printf("A~B~相位差：%.2f\r\n",Phase_A_B);	

	if(flag==1){target_phase_B += target_phase_A_B - Phase_A_B;}	

   Phase_Fix_B += target_phase_B - Pre_Phase_B;     //相位误差量 target_phase_B - Pre_Phase_B;
		 while(Phase_Fix_B>360)Phase_Fix_B-=360;
		 while(Phase_Fix_B<0)Phase_Fix_B+=360;
     
		  AD9834_Set_Phase_2(0,Phase_Fix_B);  
		           
			Counter++;
			if(Counter>=100){
			Counter=0;
			printf("t4.txt=\"%.2f度\"\xff\xff\xff",Phase_A_B-180);	//打印当前相位值
			}
			RX_Change();
		
   }

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
		    if(hadc->Instance == ADC3)
    {
        adc3_dma_complete_flag = 1;  // ADC3 采集完成，置标志位
			  HAL_TIM_Base_Stop(&htim3);
    }
		
		
}

void RX_Change(void)//0：48     d：100  f：102
{
  if(g_usart_rx_buf[0] == 100)//d:100 做包头，f：102做包尾，arr[1]以后用作模式切换,切记串口屏数据是从第三位开始的
	{  
                         //                                                  第二位是标志位
    flag=1;
		float num = 0;//蕴含信息转换后存放在num中
		float a=0;
	  float point_ten = 10;//用于递进十分位百分位
		uint8_t i = 2;//数据信息从第2位开始
		
		uint8_t point = 0;//用于区分小数点前后
		while(g_usart_rx_buf[i] != 102)//不是包尾
		{
		  if(g_usart_rx_buf[i] == 46){point=1;i++;continue;}//该位是小数点
			if(point ==1){num += (float)(g_usart_rx_buf[i]-48)/point_ten;point_ten*=10;i++;continue;}
		  else {num *=10; num +=g_usart_rx_buf[i]-48;i++;continue;}		
		}

    target_phase_A_B   = num;  //修改目标相位值
		printf("t1.txt=\"%.2f度\"\xff\xff\xff",target_phase_A_B);
		
		target_phase_A_B+=180;
		while(target_phase_B>360)target_phase_B-=360;
		
	  Rx_Delete();
	}

}
void Rx_Delete(void)
{
  for(uint8_t i=0;i<25;i++){g_usart_rx_buf[i] = 0;}
  g_usart_rx_sta =0;
}

