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

// 最大采样点数

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
void lcd_draw_lissajous(uint16_t amp1, uint16_t amp2, float phase_diff, 
                         float freq_ratio, uint32_t color);
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
    lcd_init();
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
	 Finally_Freq = Finally_Index*2.44140625*2;
	
	 printf("测量频率1：%.2f Hz\r\n",Finally_Freq);	
			
	 Finally_Index=((Index_2-1)* FFT_mag_2[Index_2-1] * FFT_mag_2[Index_2-1] + Index_2 * FFT_mag_2[Index_2] * FFT_mag_2[Index_2] + (Index_2+1)* FFT_mag_2[Index_2+1] * FFT_mag_2[Index_2+1]) /(FFT_mag_2[Index_2-1] * FFT_mag_2[Index_2-1]+FFT_mag_2[Index_2] * FFT_mag_2[Index_2]+FFT_mag_2[Index_2+1] * FFT_mag_2[Index_2+1]);
	 float Finally_Freq_2 = Finally_Index *2.44140625*2;
	 printf("测量频率2：%.2f Hz\r\n",Finally_Freq_2);	
			
			
float mag_1_prev = FFT_mag[Index-1];
float mag_1_curr = FFT_mag[Index];
float mag_1_next = FFT_mag[Index+1];

// 实部加权插值（用幅度平方作为权重）
float real_1_prev = FFT_input[2*(Index-1)];
float real_1_curr = FFT_input[2*Index];
float real_1_next = FFT_input[2*(Index+1)];

float weight_r1_prev = mag_1_prev * mag_1_prev;
float weight_r1_curr = mag_1_curr * mag_1_curr;
float weight_r1_next = mag_1_next * mag_1_next;
float weight_r1_sum = weight_r1_prev + weight_r1_curr + weight_r1_next;

float real_1 = (real_1_prev * weight_r1_prev + 
                real_1_curr * weight_r1_curr + 
                real_1_next * weight_r1_next) / weight_r1_sum;

// 虚部加权插值
float imag_1_prev = FFT_input[2*(Index-1)+1];
float imag_1_curr = FFT_input[2*Index+1];
float imag_1_next = FFT_input[2*(Index+1)+1];

float weight_i1_prev = mag_1_prev * mag_1_prev;
float weight_i1_curr = mag_1_curr * mag_1_curr;
float weight_i1_next = mag_1_next * mag_1_next;
float weight_i1_sum = weight_i1_prev + weight_i1_curr + weight_i1_next;

float imag_1 = (imag_1_prev * weight_i1_prev + 
                imag_1_curr * weight_i1_curr + 
                imag_1_next * weight_i1_next) / weight_i1_sum;

// 计算信号1的相位
float phase_1 = atan2f(imag_1, real_1) * 180.0f / PI;

// --- 信号2（倍频，2kHz）的相位计算 ---

float mag_2_prev = FFT_mag_2[Index_2-1];
float mag_2_curr = FFT_mag_2[Index_2];
float mag_2_next = FFT_mag_2[Index_2+1];

// 实部加权插值
float real_2_prev = FFT_input_2[2*(Index_2-1)];
float real_2_curr = FFT_input_2[2*Index_2];
float real_2_next = FFT_input_2[2*(Index_2+1)];

float weight_r2_prev = mag_2_prev * mag_2_prev;
float weight_r2_curr = mag_2_curr * mag_2_curr;
float weight_r2_next = mag_2_next * mag_2_next;
float weight_r2_sum = weight_r2_prev + weight_r2_curr + weight_r2_next;

float real_2 = (real_2_prev * weight_r2_prev + 
                real_2_curr * weight_r2_curr + 
                real_2_next * weight_r2_next) / weight_r2_sum;

// 虚部加权插值
float imag_2_prev = FFT_input_2[2*(Index_2-1)+1];
float imag_2_curr = FFT_input_2[2*Index_2+1];
float imag_2_next = FFT_input_2[2*(Index_2+1)+1];

float weight_i2_prev = mag_2_prev * mag_2_prev;
float weight_i2_curr = mag_2_curr * mag_2_curr;
float weight_i2_next = mag_2_next * mag_2_next;
float weight_i2_sum = weight_i2_prev + weight_i2_curr + weight_i2_next;

float imag_2 = (imag_2_prev * weight_i2_prev + 
                imag_2_curr * weight_i2_curr + 
                imag_2_next * weight_i2_next) / weight_i2_sum;

// 计算信号2的相位
float phase_2 = atan2f(imag_2, real_2) * 180.0f / PI;

uint8_t Freq_realationship = (uint8_t)((Finally_Freq_2 / Finally_Freq)+0.5);

printf("Freq_realationship = %d\r\n",Freq_realationship);
float Phase_A_B = phase_2 - Freq_realationship * phase_1;
     if(Freq_realationship==1)Phase_A_B = Phase_A_B;
else if(Freq_realationship==2)Phase_A_B = Phase_A_B -90;
else if(Freq_realationship==3)Phase_A_B = Phase_A_B +0.15;
else if(Freq_realationship==4)Phase_A_B = Phase_A_B -90 ;
else if(Freq_realationship==5)Phase_A_B = Phase_A_B -180;
else if(Freq_realationship==6)Phase_A_B = Phase_A_B -270;

while(Phase_A_B >= 360.0f) Phase_A_B -= 360.0f;
while(Phase_A_B < 0.0f) Phase_A_B += 360.0f;
    float AMP_1 = FFT_Amp/FFT_LEN*2.538;
    float AMP_2 = FFT_Amp_2/FFT_LEN*2.538;
		
		
printf("相位差：%.2f 度\r\n",Phase_A_B);
printf("信号1幅值：%.2fmv\r\n", AMP_1);
printf("信号2幅值：%.2fmv\r\n", AMP_2);


  lcd_draw_lissajous(AMP_1,AMP_2,Phase_A_B,Freq_realationship,BLUE);
	delay_ms(1000);
	
	
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


#include <math.h>
#include <stdint.h>

/**
 * @brief   绘制李萨如图形
 * @param   amp1        : 信号1的幅值 (mV)
 * @param   amp2        : 信号2的幅值 (mV)  
 * @param   phase_diff  : 信号2相对于信号1的相位差 (度)
 * @param   freq_ratio  : 频率比 f2/f1
 * @param   color       : 图形颜色
 */
void lcd_draw_lissajous(uint16_t amp1, uint16_t amp2, float phase_diff, 
                         float freq_ratio, uint32_t color)
{
    float t, step;
    int x, y, prev_x, prev_y;
    int i, total_points;
    
    // 幅值归一化 (3Vpp对应幅值1500mV)
    float a1 = (float)amp1 / 1500.0f;
    float a2 = (float)amp2 / 1500.0f;
    float phase = phase_diff * 3.14159265f / 180.0f;
    
    // 计算采样点数
    total_points = 2000;
    step = 2.0f * 3.14159265f / total_points;
    
    // 清屏
    lcd_clear(WHITE);
    
    // 绘制坐标轴
    lcd_draw_hline(0, 240, 800, 0x333333);
    for (i = 0; i < 480; i += 2) {
        lcd_draw_point(400, i, 0x333333);
    }
    
    // 初始点
    t = 0;
    prev_x = (int)(400 + a1 * sinf(t) * 350);
    prev_y = (int)(240 - a2 * sinf(freq_ratio * t + phase) * 350);
    
    // 绘制图形
    for (i = 1; i <= total_points; i++) {
        t = i * step;
        x = (int)(400 + a1 * sinf(t) * 350);
        y = (int)(240 - a2 * sinf(freq_ratio * t + phase) * 350);
        
        // 直接用画线函数
        lcd_draw_line(prev_x, prev_y, x, y, color);
        
        prev_x = x;
        prev_y = y;
    }
    
    // 标记起点
    lcd_fill_circle(prev_x, prev_y, 4, RED);
}
