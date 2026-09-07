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

#define SAMPLE_RATE   200000.0f   // 100KHz采样率
#define SIGNAL_FREQ   10000.0f    // 10KHz信号
#define SAMPLE_COUNT  4096         // 采样点数
#define NOISE_LEVEL   0.3f       // 30%噪声
#define BLOCK_SIZE 32 //调用一次arm_fir_f32 处理的采样点个数
float  arr_1[SAMPLE_COUNT] = {0};//滤波后数组
float  arr_2[SAMPLE_COUNT] = {0};//滤波后数组

const int BL = 51;
float firStateF32[BLOCK_SIZE+BL-1];// 状态缓存
const float B[51] = {
  -0.0007220591069,-0.001057500602,-0.001294314628,-0.001317842165,-0.0009444202296,
  9.839360949e-19, 0.001560610486, 0.003539497498, 0.005441842601, 0.006527714897,
   0.005984849762,  0.00319639896,-0.001962667098,-0.008891983889, -0.01618586294,
   -0.02176788636, -0.02324718796, -0.01843971945,-0.005941953976,  0.01438594796,
    0.04116404057,  0.07155539095,   0.1016660407,   0.1272164583,   0.1443465352,
     0.1503761262,   0.1443465352,   0.1272164583,   0.1016660407,  0.07155539095,
    0.04116404057,  0.01438594796,-0.005941953976, -0.01843971945, -0.02324718796,
   -0.02176788636, -0.01618586294,-0.008891983889,-0.001962667098,  0.00319639896,
   0.005984849762, 0.006527714897, 0.005441842601, 0.003539497498, 0.001560610486,
  9.839360949e-19,-0.0009444202296,-0.001317842165,-0.001294314628,-0.001057500602,
  -0.0007220591069
};

void RNG_Init(void);
uint32_t RNG_GetRandom(void);
void generate_signal(void);
void arm_fir_f32_lp(void);
int main(void)
{  
    sys_cache_enable();     	              /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */  
	  RNG_Init();
	
	
    generate_signal();
    arm_fir_f32_lp();
	  for(uint32_t i=0;i<SAMPLE_COUNT;i++)
	  {
      printf("%.2f\n",arr_2[i]);
		}
		
    while (1)
    { 
		}
}

void arm_fir_f32_lp(void)
{
 uint32_t i;
 arm_fir_instance_f32 S;
 float *inputF32,*outputF32;//初始化输入输出指针
 arm_fir_init_f32(&S,BL,(float*)&B[0],firStateF32,BLOCK_SIZE);	
	
uint32_t numBlocks = SAMPLE_COUNT / BLOCK_SIZE;
for(i = 0; i < numBlocks; i++)
	{
	    arm_fir_f32(&S, 
            &arr_1[i * BLOCK_SIZE],
            &arr_2[i * BLOCK_SIZE], 
            BLOCK_SIZE);
	}
}
void generate_signal(void)
{
    float step = 2.0f * PI * SIGNAL_FREQ / SAMPLE_RATE;
    float noise;
    uint32_t random;
    
    for(uint16_t i = 0; i < SAMPLE_COUNT; i++)
    {
        arr_1[i] = arm_sin_f32(i * step);
      
        random = RNG_GetRandom();// 生成随机噪声（使用RNG）
        noise = NOISE_LEVEL * ((float)random / 0xFFFFFFFF - 0.5f);  // 归一化到[-0.5, 0.5]
        
        
        arr_1[i] += noise;// 添加噪声
    }
}



void RNG_Init(void)// 初始化RNG
{
    __HAL_RCC_RNG_CLK_ENABLE();  // 使能RNG时钟
    RNG->CR |= RNG_CR_RNGEN;     // 使能RNG
    while(!(RNG->SR & RNG_SR_DRDY)); // 等待就绪
}


uint32_t RNG_GetRandom(void)// 获取随机数
{
    while(!(RNG->SR & RNG_SR_DRDY));
    return RNG->DR;
}
