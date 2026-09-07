#ifndef __AD9833_H
#define __AD9833_H
#include "./SYSTEM/sys/sys.h"


#define AD9833_SYSTEM_COLCK    25000000UL

 
/* WaveMode */
#define Triangle_Wave    0x2002
#define Sine_Wave  0x2028   //0x2000只有正弦波
#define Square_Wave  0x2028//方波，也有正弦波

 #define TRI_WAVE 	0  		//三角波
#define SIN_WAVE 	1		//正弦波
#define SQU_WAVE 	2		//方波
/* Registers */
 

 
/* Command Control Bits */
 
#define AD9833_B28          (1 << 13)
#define AD9833_HLB          (1 << 12)
#define AD9833_FSEL0        (0 << 11)
#define AD9833_FSEL1        (1 << 11)
#define AD9833_PSEL0        (0 << 10)
#define AD9833_PSEL1        (1 << 10)
#define AD9833_PIN_SW       (1 << 9)
#define AD9833_RESET        (1 << 8)
#define AD9833_CLEAR_RESET  (0 << 8)
#define AD9833_SLEEP1       (1 << 7)
#define AD9833_SLEEP12      (1 << 6)
#define AD9833_OPBITEN      (1 << 5)
#define AD9833_SIGN_PIB     (1 << 4)
#define AD9833_DIV2         (1 << 3)
#define AD9833_MODE         (1 << 1)


void AD9833_Init(void);

void AD9833_WriteData1(uint16_t Data);
void AD9833_WriteData2(uint16_t Data);

void AD9833_WaveSeting1(double Freq,unsigned int WaveMode,unsigned int Phase );
void AD9833_WaveSeting2(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase );

void AD9833_FreqChangeTri1(double Freq);
void AD9833_FreqChangeSine1(double Freq);




void AD9833_PhaseChange1(unsigned int Phase );



#endif  
