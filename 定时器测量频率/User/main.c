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
#include "./CMSIS/DSP/Include/arm_math.h"


/**
 * FFT长度，默认是1024点FFT 
 * 可选范围为: 16, 64, 256, 1024.
 */
#define FFT_LENGTH    4096

float g_fft_inputbuf[FFT_LENGTH * 2];       /* FFT输入数组 */
float g_fft_outputbuf[FFT_LENGTH];          /* FFT输出数组 */

uint8_t g_timeout;                          /* 计时全局变量 */

int main(void)
{  
    float time;
    char buf[50];
    arm_cfft_radix4_instance_f32 scfft;
    uint8_t key, t = 0;
    uint16_t i;
  
    sys_cache_enable();                     /* 使能L1-Cache */
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);     /* 设置时钟, 480Mhz */
    delay_init(480);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化USART */ 
    led_init();                             /* 初始化LED */
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init();                           /* 初始化SDRAM */
    lcd_init();                             /* 初始化LCD */
    key_init();                             /* 初始化按键 */  
    //timx_int_init(65535, 240 - 1);          /* 1Mhz计数频率,设置自动重载值为65536 */
    //TIMX_INT->CNT = 0;                 /* 重设定时器TIMX的计数器值 */
    //g_timeout = 0;
   // arm_cfft_radix4_init_f32(&scfft, FFT_LENGTH, 0, 1);         /* 初始化scfft结构体，设定FFT相关参数 */
//            arm_cfft_radix4_f32(&scfft, g_fft_inputbuf);                    /* FFT计算（基4） */
//            
//            time = TIMX_INT->CNT + (uint32_t)g_timeout * 65536;             /* 计算所用时间 */
//            sprintf((char *)buf, "%0.3fms\r\n", time / 1000);
//            lcd_show_string(30 + 12 * 8, 160, 100, 16, 16, buf, BLUE);      /* 显示运行时间 */
//            
//            arm_cmplx_mag_f32(g_fft_inputbuf, g_fft_outputbuf, FFT_LENGTH); /* 把运算结果复数求模得幅值 */
    while (1)
    {        
       printf("ok\r\n");delay_ms(1000);
			
			
			
        

    }
}




