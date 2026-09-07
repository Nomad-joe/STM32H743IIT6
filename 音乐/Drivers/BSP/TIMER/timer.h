/**
 ****************************************************************************************************
 * @file        timer.h
 * @version     V1.0
 * @brief       定时器中断 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 ****************************************************************************************************
 */

#ifndef __TIM_H
#define __TIM_H

#include "./SYSTEM/sys/sys.h"

#define NOTE_L_DO    131
#define NOTE_L_RE    147
#define NOTE_L_MI    165
#define NOTE_L_FA    175
#define NOTE_L_SO    196
#define NOTE_L_LA    220
#define NOTE_L_SI    247

/* 中音区 */
#define NOTE_DO      262
#define NOTE_RE      294
#define NOTE_MI      330
#define NOTE_FA      349
#define NOTE_SO      392
#define NOTE_LA      440
#define NOTE_SI      494

/* 高音区 */
#define NOTE_H_DO    523
#define NOTE_H_RE    587
#define NOTE_H_MI    659
#define NOTE_H_FA    698
#define NOTE_H_SO    784
#define NOTE_H_LA    880
#define NOTE_H_SI    988

/* 休止符（静音） */
#define NOTE_PAUSE   0
/******************************************************************************************/
/* 定时器 定义 */

extern TIM_HandleTypeDef htim2;
void MX_TIM2_Init(void);
void Music_Tone(uint32_t freq, uint8_t duty, uint32_t time_ms);
#endif




