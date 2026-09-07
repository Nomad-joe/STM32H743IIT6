#ifndef __AD7606_H
#define __AD7606_H
#include "./SYSTEM/sys/sys.h"


#define AD7606B_Range    	10000.0	// AD7606B量程±10V
#define AD7606B_Range_2_5V		0x00	// ±2.5V量程
#define AD7606B_Range_5V		0x11	// ±5V量程
#define AD7606B_Range_10V		0x22	// ±10V量程

/*********************************引脚声明*********************************************/

////#define AD7606B_CNT_L      	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
////#define AD7606B_CNT_H     	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
//#define AD7606B_CNT_H     	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET)
////CNT 使用 PI6的PWM代替

//#define AD7606B_RST_L      	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET)
//#define AD7606B_RST_H     	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET)


//#define AD7606B_BUSY        HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_8)

//#define AD7606B_CS_L      	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_RESET)
//#define AD7606B_CS_H     	 HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_SET)




// 使用寄存器操作GPIO，提高速度
#define AD7606B_CS_L()     do { GPIOI->BSRR = (uint32_t)GPIO_PIN_9 << 16U; } while(0)  // 复位
#define AD7606B_CS_H()     do { GPIOI->BSRR = GPIO_PIN_9; } while(0)                      // 置位

#define AD7606B_RST_L()    do { GPIOE->BSRR = (uint32_t)GPIO_PIN_4 << 16U; } while(0)
#define AD7606B_RST_H()    do { GPIOE->BSRR = GPIO_PIN_4; } while(0)

#define AD7606B_BUSY       ((GPIOI->IDR & GPIO_PIN_8) != 0)  // 读取BUSY状态

//#define AD7606B_FDA        	HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6)
#define SPI4_WRITE_READ(data) ({ \
    uint16_t __ret; \
    while((SPI4->SR & SPI_SR_TXE) == 0); \
    SPI4->DR = (data); \
    while((SPI4->SR & SPI_SR_RXNE) == 0); \
    __ret = SPI4->DR; \
    __ret; \
})


extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi4_rx;
void MX_SPI4_Init(void);
uint16_t spi1_read_write_16bit(uint16_t txdata);

extern void AD7606B_Init(void);
extern uint8_t AD7606B_Read_Reg(uint8_t Addr);
extern void AD7606B_Write_Reg(uint8_t Addr, uint8_t Data);
extern void AD7606B_Read_AD_Data(int16_t * data);
extern void PI8_EXTI_Init(void);//BUSY
extern void ad7606_Reset(void);
extern void AD7606B_Read_AD_Data(int16_t *data);

#endif
