#ifndef __AD7606_H
#define __AD7606_H
#include "./SYSTEM/sys/sys.h"


#define AD7606B_Range    	10000.0	// AD7606B量程±10V
#define AD7606B_Range_2_5V		0x00	// ±2.5V量程
#define AD7606B_Range_5V		0x11	// ±5V量程
#define AD7606B_Range_10V		0x22	// ±10V量程

/*********************************引脚声明*********************************************/

#define AD7606B_CNT_L      	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define AD7606B_CNT_H     	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

#define AD7606B_RST_L      	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET)
#define AD7606B_RST_H     	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET)


#define AD7606B_BUSY        HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_8)

#define AD7606B_CS_L      	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_RESET)
#define AD7606B_CS_H     	 HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_SET)

//#define AD7606B_FDA        	HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6)



extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi4_rx;
void MX_SPI4_Init(void);
uint16_t spi1_read_write_16bit(uint16_t txdata);

extern void AD7606B_Init(void);
extern void AD7606B_Start_Convst(void);
extern uint8_t AD7606B_Read_Reg(uint8_t Addr);
extern void AD7606B_Write_Reg(uint8_t Addr, uint8_t Data);
extern void AD7606B_Read_AD_Data(int16_t * data);



#endif
