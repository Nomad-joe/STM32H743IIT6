#ifndef __AD9834_H
#define __AD9834_H

#include "./SYSTEM/sys/sys.h"


/* AD9834系统晶振 */
#define AD9834_SYSTEM_COLCK     75000000UL


#define Triangle_Wave    0x2002
#define Sine_Wave  0x2028   //0x2000只有正弦波
#define Square_Wave  0x2028//方波，也有正弦波

#define TRI_WAVE 	0  		//三角波
#define SIN_WAVE 	1		//正弦波
#define SQU_WAVE 	2		//方波

#define FREQ_0      0
#define FREQ_1      1

#define DB15        0
#define DB14        0
#define DB13        B28
#define DB12        HLB
#define DB11        FSEL
#define DB10        PSEL
#define DB9         PIN_SW
#define DB8         RESET
#define DB7         SLEEP1
#define DB6         SLEEP12
#define DB5         OPBITEN
#define DB4         SIGN_PIB
#define DB3         DIV2
#define DB2         0
#define DB1         MODE
#define DB0         0

#define CONTROL_REGISTER    (DB15<<15)|(DB14<<14)|(DB13<<13)|(DB12<<12)|(DB11<<11)|(DB10<<10)\
    |(DB9<<9)|(DB8<<8)|(DB7<<7)|(DB6<<6)|(DB5<<5)|(DB4<<4)|(DB3<<3)|(DB2<<2)|(DB1<<1)|(DB0<<0)

/* AD9834函数声明 */
void AD9834_Init(void);
extern void AD9834_Write_16Bits(unsigned int Data) ;  //写一个字到AD9834
extern void AD9834_Select_Wave(unsigned int initdata) ; //选择输出波形，设置控制字
extern void AD9834_Set_Freq(unsigned char freq_chanel, unsigned long freq) ;//选择输出寄存器和输出频率
extern void AD9834_Set_Phase(unsigned char CH,float Phase);//选择输出寄存器和相位

void AD9834_AmpSet(unsigned char amp);
//设置整合到一个函数中，方便使用
void AD9834_WaveSeting(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase );

void AD9834_SetPSK(float Phase1,float Phase2,unsigned long Freq);//正弦波相位调制
void AD9834_SetFSK(unsigned long Freq1,unsigned long Freq2);//正弦波频率调制
#endif
