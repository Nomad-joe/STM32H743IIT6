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
/**
 * FFT长度，默认是1024点FFT 
 * 可选范围为: 16, 64, 256, 1024.
 */
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len256;
#define SAMPLE_RATE 8000
#define FFT_LEN  256
uint16_t DMA_Buffer[FFT_LEN] = {0};
float ADC_float [FFT_LEN] = {0};
float FFT_input[2*FFT_LEN] = {0};
float FFT_mag[FFT_LEN] ={0};
uint32_t Index = 0; //mag数组中最大值的下标
uint32_t Index2 =0;
uint32_t Index3 =0;
float Finally_Index ;
float Finally_Freq;
float Finally_Amp;
float FFT_Amp;

uint8_t fft_flag=0;
uint16_t fall_pot[480] = {0};  // 峰值下落数组
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
    lcd_init();
   MX_ADC1_Init();//初始化adc PA0
   MX_TIM3_Init();//初始化adc所需的timer 频率为8K
	 HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//ADC校正函数
	 delay_ms(20);
      // 初始化峰值下落数组
    for(uint16_t i = 0; i < 480; i++) {
        fall_pot[i] = 800;  
    }
   
    while (1)
    {  
 			 HAL_TIM_Base_Start(&htim3);     
       HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_Buffer,FFT_LEN);//ADC开始采集，DMA开启搬运
		  while(fft_flag==0){};
			fft_flag=0;
			HAL_TIM_Base_Stop(&htim3);
			for(uint16_t i=0;i<FFT_LEN;i++)
			{
			  ADC_float[i] = (float)DMA_Buffer[i]*3.3f/65535;
				ADC_float[i] -=1.25;//减去直流偏置
				ADC_float[i] *= 0.5f*(1-arm_cos_f32(2*PI*(i)/(FFT_LEN - 1)));  //时域加窗+转变为复信号
				FFT_input[2*i] = ADC_float[i];
				FFT_input[2*i+1] = 0;
			}

       arm_cfft_f32(&arm_cfft_sR_f32_len256, FFT_input, 0, 1);
       arm_cmplx_mag_f32(FFT_input, FFT_mag, FFT_LEN);
			FFT_mag[0] = 0;
			arm_max_f32(FFT_mag,FFT_LEN/2,&FFT_Amp,&Index);
			 lcd_clear(WHITE);
//       for(uint8_t i=0;i<128;i++)
//       {
//			   printf("%.2f\r\n",FFT_mag[i]);
//			 }			
			        for(uint16_t i = 0; i < 480; i++)
        {
            // 对数频率映射
            float min_freq = 10.0f;
            float max_freq = 4000.0f;
            
            float log_min = log10f(min_freq);
            float log_max = log10f(max_freq);
            float freq = powf(10.0f, log_min + ((float)i / 480.0f) * (log_max - log_min));
            
            // 频率转FFT索引
            uint16_t freq_idx = (uint16_t)(freq * FFT_LEN / SAMPLE_RATE);
            if(freq_idx > 127) freq_idx = 127;
            if(freq_idx < 1)   freq_idx = 1;
            
            float mag_value = FFT_mag[freq_idx]*10;
            
            int32_t height = (int32_t)(mag_value);  
            if(height > 800) height = 800;
            if(height < 0) height = 0;
            
            int32_t y_top = 800 - height;
            if(y_top < 0) y_top = 0;
            
            // 颜色设置
            uint16_t color;
            if(height > 600) color = RED;
            else if(height > 400) color = YELLOW;
            else if(height > 200) color = GREEN;
            else color = CYAN;
            
            // 绘制频谱柱
            lcd_draw_line(i, 800, i, y_top, color);
            
            // 峰值下落效果
            if(fall_pot[i] < 800) {
                lcd_draw_point(i, fall_pot[i], WHITE);
                if(i > 0) lcd_draw_point(i-1, fall_pot[i], WHITE);
                if(i < 479) lcd_draw_point(i+1, fall_pot[i], WHITE);
            }
            
            if(fall_pot[i] >= y_top) {
                fall_pot[i] = y_top;
            }
            
            if(fall_pot[i] < 800) {
                lcd_draw_point(i, fall_pot[i], BLUE);
                if(i > 0) lcd_draw_point(i-1, fall_pot[i], BLUE);
                if(i < 479) lcd_draw_point(i+1, fall_pot[i], BLUE);
            }
            
            if(fall_pot[i] < 800) {
                fall_pot[i] += 2;
            }
            if(fall_pot[i] >= 800) {
                fall_pot[i] = 800;
            }
        }
         
    }

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc ==&hadc1){
    fft_flag = 1;
	}
}


