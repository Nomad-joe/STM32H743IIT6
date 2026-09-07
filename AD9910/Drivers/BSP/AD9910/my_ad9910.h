#ifndef __AD9910_H
#define __AD9910_H

#include "./SYSTEM/sys/sys.h"

#define uchar unsigned char
#define uint  unsigned int	
#define ulong  unsigned long int

/* 时钟端口定义 */
#define AD9910_SDIO_PORT      GPIOH
#define AD9910_SDIO_PIN       GPIO_PIN_15

#define UP_DAT_PORT           GPIOI
#define UP_DAT_PIN            GPIO_PIN_1

#define PROFILE1_PORT         GPIOA
#define PROFILE1_PIN          GPIO_PIN_15

#define MAS_REST_PORT         GPIOD
#define MAS_REST_PIN          GPIO_PIN_2

#define SCLK_PORT             GPIOD
#define SCLK_PIN              GPIO_PIN_4

#define OSK_PORT              GPIOD
#define OSK_PIN               GPIO_PIN_6

#define PROFILE0_PORT         GPIOI
#define PROFILE0_PIN          GPIO_PIN_3

#define PROFILE2_PORT         GPIOC
#define PROFILE2_PIN          GPIO_PIN_11

#define CS_PORT               GPIOG
#define CS_PIN                GPIO_PIN_9

/* 完全按照你 LED 的格式写 —— 最稳定、最标准 */
#define AD9910_SDIO(x)   do{ x ? \
                      HAL_GPIO_WritePin(AD9910_SDIO_PORT, AD9910_SDIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(AD9910_SDIO_PORT, AD9910_SDIO_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define UP_DAT(x)   do{ x ? \
                      HAL_GPIO_WritePin(UP_DAT_PORT, UP_DAT_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(UP_DAT_PORT, UP_DAT_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define PROFILE1(x)   do{ x ? \
                      HAL_GPIO_WritePin(PROFILE1_PORT, PROFILE1_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(PROFILE1_PORT, PROFILE1_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define MAS_REST(x)   do{ x ? \
                      HAL_GPIO_WritePin(MAS_REST_PORT, MAS_REST_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(MAS_REST_PORT, MAS_REST_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define SCLK(x)   do{ x ? \
                      HAL_GPIO_WritePin(SCLK_PORT, SCLK_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(SCLK_PORT, SCLK_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define OSK(x)   do{ x ? \
                      HAL_GPIO_WritePin(OSK_PORT, OSK_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(OSK_PORT, OSK_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define PROFILE0(x)   do{ x ? \
                      HAL_GPIO_WritePin(PROFILE0_PORT, PROFILE0_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(PROFILE0_PORT, PROFILE0_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define PROFILE2(x)   do{ x ? \
                      HAL_GPIO_WritePin(PROFILE2_PORT, PROFILE2_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(PROFILE2_PORT, PROFILE2_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define CS(x)   do{ x ? \
                      HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); \
                  }while(0)






////#define AD9910_CSN_Set CS = 1
////#define AD9910_CSN_Clr CS = 0

////#define AD9910_IUP_Set UP_DAT = 1     
////#define AD9910_IUP_Clr UP_DAT = 0

typedef enum {
	TRIG_WAVE = 0,
	SQUARE_WAVE,
	SINC_WAVE,
} AD9910_WAVE_ENUM;

void AD9110_IOInit(void);
void Init_AD9910(void);
void AD9910_FreWrite(ulong Freq);										//写频率
void AD9910_AmpWrite(uint16_t Amp);


void AD9910_RAM_WAVE_Set(AD9910_WAVE_ENUM wave);



#endif 