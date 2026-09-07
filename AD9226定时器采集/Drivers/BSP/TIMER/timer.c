/**
 ****************************************************************************************************
 * @file        timer.c
 * @version     V1.0
 * @brief       定时器中断 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 ****************************************************************************************************
 */

#include "./BSP/LED/led.h"
#include "./BSP/TIMER/timer.h"
#include "./SYSTEM/delay/delay.h"

uint16_t AD9226_Data[FFT_LENGTH+20];
volatile uint16_t k = 0;
volatile uint8_t Time_Flag;
TIM_HandleTypeDef TIM3_Handler; 
void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();                            //使能TIM3时钟
	
    TIM3_Handler.Instance=TIM3;                          	//通用定时器3
    TIM3_Handler.Init.Prescaler=1-1;                     	//分频系数
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    	//向上计数器
    TIM3_Handler.Init.Period=120-1;                        	//自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;	//时钟分频因子
    HAL_TIM_Base_Init(&TIM3_Handler);

    HAL_NVIC_SetPriority(TIM3_IRQn,1,0);    //设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断
    HAL_TIM_Base_Start_IT(&TIM3_Handler);   //使能定时器3和定时器3更新中断：TIM_IT_UPDATE
}

/*******************************************************************
@ function: void TIM3_IRQHandler(void)
@ 函数功能: 定时器3中断服务函数
@ 入口参数: 无
@ 返回参数: 无
@ 注意事项: 无
********************************************************************/
void TIM3_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&TIM3_Handler,TIM_IT_UPDATE);
	  GPIOH->BSRR = GPIO_PIN_9;
    AD9226_Data[k] = GPIOI->IDR;
    GPIOH->BSRR  = (uint32_t)GPIO_PIN_9 << 16U;
    k++;
    if(k==(FFT_LENGTH+20))
    {
        Time_Flag = 1;
        k = 0;
        HAL_TIM_Base_Stop_IT(&TIM3_Handler);
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
    }
}
void AD9226_Read_Data(uint8_t mode)
{  
	uint16_t *pb = AD9226_Data;
	if(mode==0){
   TIM3_Init();
   while(Time_Flag == 0);
   Time_Flag = 0; 
  }
	else if(mode ==1)
	{
	  k = FFT_LENGTH + 20;
    pb = AD9226_Data;
    do
    {
     GPIOH->BSRR = GPIO_PIN_9;
    *(pb++) = ((GPIOI->IDR)&0x0FFF);
	  GPIOH->BSRR  = (uint32_t)GPIO_PIN_9 << 16U;
    }while(k--);
	}
		
}
