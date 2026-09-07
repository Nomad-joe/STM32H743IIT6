 
#include "./BSP/AD9833/my_ad9833.h"


// HAL库方式重写引脚操作宏

#define FSYNC_1_0()    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET)
#define FSYNC_1_1()    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET)

#define SCK_1_0()      HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET)
#define SCK_1_1()      HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET)

#define DAT_1_0()      HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET)
#define DAT_1_1()      HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET)





#define FSYNC_2_0()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define FSYNC_2_1()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

#define SCK_2_0()      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)
#define SCK_2_1()      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)

#define DAT_2_0()      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#define DAT_2_1()      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET)

/**************************************
*   函 数 名: ad9833_init
*   功能说明: ad9833初始化
*   形    参: 无
*   返 回 值: 无
*************************************/
void AD9833_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_GPIOI_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
}
 
/**************************************
*   函 数 名: AD9833_Delay
*   功能说明: ad9833延迟
*   形    参: 无
*   返 回 值: 无
*************************************/
static void AD9833_Delay(void)
{
	unsigned int i;
	for (i = 0; i < 10; i++);
}
/**************************************
*   函 数 名: ad9833_write_data
*   功能说明: ad9833写入16位数据
*   形    参: txdata：待写入的16位数据
*   返 回 值: 无
*************************************/
void AD9833_WriteData1(uint16_t Data)
{
    unsigned char i = 0 ;
    
        SCK_1_1();
        FSYNC_1_1();
        FSYNC_1_0();
    for(i=0 ;i<16 ;i++) 
    {
        if(Data & 0x8000)
        {DAT_1_1() ;
        AD9833_Delay();}
        else
        {DAT_1_0() ;}
        
            SCK_1_0();
        AD9833_Delay();
            Data <<= 1 ;
        SCK_1_1();
    }
    
    FSYNC_1_1();
}
void AD9833_WriteData2(uint16_t txdata)
{
    unsigned char i = 0 ;
    SCK_2_1();
    FSYNC_2_1();
    FSYNC_2_0();
    //写16位数据
    for(i=0;i<16;i++)
    {
        
        if (txdata & 0x8000){
           DAT_2_1();
            }
        else{
           DAT_2_0();}
        
        SCK_2_0();
        AD9833_Delay();

            txdata<<=1;
        SCK_2_0();
    }
    FSYNC_2_1();
}





/*
*********************************************************************************************************
*	函 数 名: AD9834_WaveSeting
*	功能说明: 向SPI总线发送16个bit数据，设置功能整合到一个函数中，方便调用
*	形    参: 1.Freq: 频率值, 0.1 hz - 37.5Mhz
			  2.Freq_SFR: 0 或 1
			  3.WaveMode: TRI_WAVE(三角波),SIN_WAVE(正弦波),SQU_WAVE(方波)
			  4.Phase : 波形的初相位,0-360
*	返 回 值: 无
*********************************************************************************************************
*/ 
void AD9833_WaveSeting1(double Freq,unsigned int WaveMode,unsigned int Phase )
{

		int frequence_LSB,frequence_MSB,Phs_data;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;
        Freq=Freq*1.07411f;
		/*********************************计算频率的16进制值***********************************/
		frequence_mid=268435456/AD9833_SYSTEM_COLCK;//适合25M晶振
		//如果时钟频率不为25MHZ，修改该处的频率值，单位MHz ，AD9833最大支持25MHz
     //   Freq=Freq/1.072367515;
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1;
		frequence_DATA=frequence_DATA*frequence_mid;
		frequence_hex=frequence_DATA;  //这个frequence_hex的值是32位的一个很大的数字，需要拆分成两个14位进行处理；
		frequence_LSB=frequence_hex; //frequence_hex低16位送给frequence_LSB
		frequence_LSB=frequence_LSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位
		frequence_MSB=frequence_hex>>14; //frequence_hex高16位送给frequence_HSB
		frequence_MSB=frequence_MSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位

	  Phs_data = Phs_data * 11.3777778 + 0.3;	
		
		AD9833_WriteData1(0x0100); //复位AD9833,即RESET位为1
		AD9833_WriteData1(0x2100); //选择数据一次写入，B28位和RESET位为1

			Phs_data=Phase|0xC000;	//相位值
		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			 //使用频率寄存器0输出波形
			AD9833_WriteData1(frequence_LSB); //L14，选择频率寄存器0的低14位数据输入
			AD9833_WriteData1(frequence_MSB); //H14 频率寄存器的高14位数据输入
			AD9833_WriteData1(Phs_data);	//设置相位
            AD9833_WriteData1(0x2000); /**设置FSELECT位为0，芯片进入工作状态,频率寄存器0输出波形**/

		if(WaveMode==TRI_WAVE) //输出三角波波形
		 	AD9833_WriteData1(0x2002); 
		if(WaveMode==SQU_WAVE)	//输出方波波形
			AD9833_WriteData1(0x2028); 
		if(WaveMode==SIN_WAVE)	//输出正弦波形
			AD9833_WriteData1(0x2000); 
}


void AD9833_FreqChangeTri1(double Freq)
{
		int frequence_LSB,frequence_MSB;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;
        Freq=Freq*1.07411f;

		/*********************************计算频率的16进制值***********************************/
		frequence_mid=268435456/AD9833_SYSTEM_COLCK;//适合25M晶振
		//如果时钟频率不为25MHZ，修改该处的频率值，单位MHz ，AD9833最大支持25MHz
        Freq=Freq/1.072367515;
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1;
		frequence_DATA=frequence_DATA*frequence_mid;
    frequence_hex=frequence_DATA;  //这个frequence_hex的值是32位的一个很大的数字，需要拆分成两个14位进行处理；
    frequence_LSB=frequence_hex; //frequence_hex低16位送给frequence_LSB
    frequence_LSB=frequence_LSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位
    frequence_MSB=frequence_hex>>14; //frequence_hex高16位送给frequence_HSB
    frequence_MSB=frequence_MSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位

		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			AD9833_WriteData1(frequence_LSB); //L14，选择频率寄存器0的低14位数据输入
			AD9833_WriteData1(frequence_MSB); //H14 频率寄存器的高14位数据输入
            AD9833_WriteData1(0x2000); /**设置FSELECT位为0，芯片进入工作状态,频率寄存器0输出波形**/
            AD9833_WriteData1(0x2002); 

}


void AD9833_FreqChangeSine1(double Freq)
{
		int frequence_LSB,frequence_MSB;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;
        Freq=Freq*1.07411f;

		/*********************************计算频率的16进制值***********************************/
		frequence_mid=268435456/AD9833_SYSTEM_COLCK;//适合25M晶振
		//如果时钟频率不为25MHZ，修改该处的频率值，单位MHz ，AD9833最大支持25MHz
        Freq=Freq/1.072367515;
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1;
		frequence_DATA=frequence_DATA*frequence_mid;
    frequence_hex=frequence_DATA;  //这个frequence_hex的值是32位的一个很大的数字，需要拆分成两个14位进行处理；
    frequence_LSB=frequence_hex; //frequence_hex低16位送给frequence_LSB
    frequence_LSB=frequence_LSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位
    frequence_MSB=frequence_hex>>14; //frequence_hex高16位送给frequence_HSB
    frequence_MSB=frequence_MSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位
       

		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			AD9833_WriteData1(frequence_LSB); //L14，选择频率寄存器0的低14位数据输入
			AD9833_WriteData1(frequence_MSB); //H14 频率寄存器的高14位数据输入
      AD9833_WriteData1(0x2000); /**设置FSELECT位为0，芯片进入工作状态,频率寄存器0输出波形**/
      
}








void AD9833_PhaseChange1(unsigned int Phase )
{

		int Phs_data;
        
	  Phs_data = Phs_data * 11.3777778 + 0.3;	
		
			Phs_data=Phase|0xC000;	//相位值
		 
			AD9833_WriteData1(Phs_data);	//设置相位
      AD9833_WriteData1(0x2000); /**设置FSELECT位为0，芯片进入工作状态,频率寄存器0输出波形**/

}










void AD9833_WaveSeting2(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase )
{

		int frequence_LSB,frequence_MSB,Phs_data;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;

		/*********************************计算频率的16进制值***********************************/
		frequence_mid=268435456/AD9833_SYSTEM_COLCK;//适合25M晶振
		//如果时钟频率不为25MHZ，修改该处的频率值，单位MHz ，AD9833最大支持25MHz
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1;
		frequence_DATA=frequence_DATA*frequence_mid;
		frequence_hex=frequence_DATA;  //这个frequence_hex的值是32位的一个很大的数字，需要拆分成两个14位进行处理；
		frequence_LSB=frequence_hex; //frequence_hex低16位送给frequence_LSB
		frequence_LSB=frequence_LSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位
		frequence_MSB=frequence_hex>>14; //frequence_hex高16位送给frequence_HSB
		frequence_MSB=frequence_MSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位

	  Phs_data = Phs_data * 11.3777778 + 0.3;	
		
//		AD9833_WriteData2(0x0100); //复位AD9833,即RESET位为1
		AD9833_WriteData2(0x2100); //选择数据一次写入，B28位和RESET位为1

		if(Freq_SFR==0)				  //把数据设置到设置频率寄存器0
		{
//			Phs_data=Phase|0xC000;	//相位值
		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			 //使用频率寄存器0输出波形
			AD9833_WriteData2(frequence_LSB); //L14，选择频率寄存器0的低14位数据输入
			AD9833_WriteData2(frequence_MSB); //H14 频率寄存器的高14位数据输入
//			AD9833_WriteData2(Phs_data);	//设置相位
			//AD9833_Write(0x2000); /**设置FSELECT位为0，芯片进入工作状态,频率寄存器0输出波形**/
	    }
		if(Freq_SFR==1)				//把数据设置到设置频率寄存器1
		{
//			Phs_data=Phase|0xE000;	//相位值
			 frequence_LSB=frequence_LSB|0x8000;
			 frequence_MSB=frequence_MSB|0x8000;
			//使用频率寄存器1输出波形
			AD9833_WriteData2(frequence_LSB); //L14，选择频率寄存器1的低14位输入
			AD9833_WriteData2(frequence_MSB); //H14 频率寄存器1为
//			AD9833_WriteData2(Phs_data);	//设置相位
			//AD9833_Write(0x2800); /**设置FSELECT位为0，设置FSELECT位为1，即使用频率寄存器1的值，芯片进入工作状态,频率寄存器1输出波形**/
		}

		if(WaveMode==TRI_WAVE) //输出三角波波形
		 	AD9833_WriteData2(0x2002); 
		if(WaveMode==SQU_WAVE)	//输出方波波形
			AD9833_WriteData2(0x2028); 
		if(WaveMode==SIN_WAVE)	//输出正弦波形
			AD9833_WriteData2(0x2000); 
        
    
}
