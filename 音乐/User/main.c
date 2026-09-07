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
 
   MX_TIM2_Init();

   Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_SO, 30, 400);  // 5
    Music_Tone(NOTE_SO, 30, 400);  // 5
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_MI, 30, 800);  // 3 - 延长
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_RE, 30, 800);  // 2 - 延长
    
    // ========== 第二句 ==========
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_SO, 30, 400);  // 5
    Music_Tone(NOTE_SO, 30, 400);  // 5
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_RE, 30, 800);  // 2 - 延长
    Music_Tone(NOTE_DO, 30, 800);  // 1 - 延长
    
    // ========== 第三句 ==========
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_MI, 30, 400);  // 3
    
    // ========== 第四句 ==========
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_DO, 30, 800);  // 1 - 结束延长
    
    // ========== 第五句（重复加强） ==========
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_SO, 30, 400);  // 5
    Music_Tone(NOTE_SO, 30, 400);  // 5
    Music_Tone(NOTE_FA, 30, 400);  // 4
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_DO, 30, 400);  // 1
    Music_Tone(NOTE_RE, 30, 400);  // 2
    Music_Tone(NOTE_MI, 30, 400);  // 3
    Music_Tone(NOTE_RE, 30, 800);  // 2 - 延长
    Music_Tone(NOTE_DO, 30, 800);  // 1 - 结束
	
	
    while (1)
    {  
//			playTone(440, 1000);   // 播放中央C (440Hz)，持续1秒
//       delay_ms(200);         // 间隔200ms
        
//        playTone(523, 500);    // 播放C5 (523Hz)，持续0.5秒
//        delay_ms(200);
//        
//        playTone(587, 500);    // 播放D5 (587Hz)
//        delay_ms(200);
//        
//        playTone(659, 500);    // 播放E5 (659Hz)
//        delay_ms(200);
//        
//        playTone(698, 500);    // 播放F5 (698Hz)
//        delay_ms(200);
//        
//        playTone(784, 500);    // 播放G5 (784Hz)
//        delay_ms(200);
//        
//        playTone(880, 500);    // 播放A5 (880Hz)
//        delay_ms(200);
//        
//        playTone(988, 500);    // 播放B5 (988Hz)
//        delay_ms(500);
			
			
			
			
		}
}





