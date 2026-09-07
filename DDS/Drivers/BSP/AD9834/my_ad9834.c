#include "./BSP/AD9834/my_ad9834.h"

  #define PORT_CS		GPIOH
	#define PIN_CS		GPIO_PIN_15  //数字电位器片选
	
	#define PORT_FSYNC	GPIOI
	#define PIN_FSYNC	GPIO_PIN_1

	#define PORT_SCK	GPIOI
	#define PIN_SCK		GPIO_PIN_3

	#define PORT_DAT	GPIOA
	#define PIN_DAT		GPIO_PIN_15

	#define PORT_RESET	GPIOC
	#define PIN_RESET		GPIO_PIN_11

//****************************************************************
	#define CS_0()		    HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_RESET)
	#define CS_1()		    HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_SET)	


	#define FSYNC_0()		HAL_GPIO_WritePin(PORT_FSYNC, PIN_FSYNC,GPIO_PIN_RESET)
	#define FSYNC_1()		HAL_GPIO_WritePin(PORT_FSYNC, PIN_FSYNC,GPIO_PIN_SET)

	#define SCK_0()		    HAL_GPIO_WritePin(PORT_SCK, PIN_SCK,GPIO_PIN_RESET)
	#define SCK_1()		    HAL_GPIO_WritePin(PORT_SCK, PIN_SCK,GPIO_PIN_SET)

	#define DAT_0()		    HAL_GPIO_WritePin(PORT_DAT, PIN_DAT,GPIO_PIN_RESET)
	#define DAT_1()		    HAL_GPIO_WritePin(PORT_DAT, PIN_DAT,GPIO_PIN_SET)	

	#define RESET_0()		HAL_GPIO_WritePin(PORT_RESET, PIN_RESET,GPIO_PIN_RESET)
	#define RESET_1()		HAL_GPIO_WritePin(PORT_RESET, PIN_RESET,GPIO_PIN_SET)	

//初始化AD9834 GPIO

void AD9834_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  
  

  
  //CS片选
  HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = PIN_CS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_CS, &GPIO_InitStruct);

  
  //FSYNC
  HAL_GPIO_WritePin(PORT_FSYNC, PIN_FSYNC,GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = PIN_FSYNC;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_FSYNC, &GPIO_InitStruct);
  
  //SCK
  HAL_GPIO_WritePin(PORT_SCK,PIN_SCK ,GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = PIN_SCK;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_SCK, &GPIO_InitStruct);
  
  //DAT
  HAL_GPIO_WritePin(PORT_DAT,PIN_DAT,GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = PIN_DAT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_DAT, &GPIO_InitStruct);
  
  //RESET
  HAL_GPIO_WritePin(PORT_RESET,PIN_RESET,GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = PIN_RESET;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PORT_RESET, &GPIO_InitStruct);
  
  
  
  
    CS_1();
	FSYNC_1();
	SCK_1();
	DAT_1();
	RESET_1();

}




/*
*********************************************************************************************************
*	函 数 名: AD9833_Delay
*	功能说明: 时钟延时
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AD9834_Delay(void)
{
	unsigned int i;
	for (i = 0; i < 10; i++);
}



/****************************************************************
函数名: AD9834_Write_16Bits
功能: 向AD9834写入16位数据
参数: data --  要写入的16位数据
返回值: 无
*****************************************************************/
void AD9834_Write_16Bits(unsigned int Data)
{
    unsigned char i = 0 ;
    
   SCK_1();
   FSYNC_0();
    for(i=0 ;i<16 ;i++) 
    {
        if(Data & 0x8000)
          DAT_1() ;
        else
          DAT_0() ;
        
        SCK_0();
        Data <<= 1 ;
        SCK_1();
    }
    
 FSYNC_1();
}
/***********************************************************************************
函数名称:AD9834_Select_Wave
功能:软件的控制字设置
    --------------------------------------------------
    IOUT正弦波 ,SIGNBITOUT方波 ,写FREQREG0 ,写PHASE0
    ad9834_write_16bit(0x2028)   一次性写FREQREG0
    ad9834_write_16bit(0x0028)   单独改写FREQREG0的LSB,一般不用
    ad9834_write_16bit(0x1028)   单独改写FREQREG0的MSB,一般不用
    --------------------------------------------------
    IOUT三角波 ,写PHASE0
    ad9834_write_16bit(0x2002)   一次性写FREQREG0
    ad9834_write_16bit(0x0002)   单独改写FREQREG0的LSB,一般不用
    ad9834_write_16bit(0x1008)   单独改写FREQREG0的MSB,一般不用
参数:initdata -- 控制字
返回值: 无
************************************************************************************/
void AD9834_Select_Wave(unsigned int initdata)
{
    FSYNC_1();
    SCK_1();
    
    RESET_1();
    RESET_1();
    RESET_0();
    //AD9834_Write_16Bits(0x0100); //复位
    AD9834_Write_16Bits(initdata);
}

void AD9834_SetFSK(unsigned long Freq1,unsigned long Freq2)//正弦波频率调制
{
    FSYNC_1();
    SCK_1();
    
    RESET_1();
    RESET_1();
    RESET_0();
    AD9834_Write_16Bits(0x0100); //复位
    AD9834_Write_16Bits(0x2228);
	  AD9834_Set_Freq(FREQ_0, Freq1);
	  AD9834_Set_Freq(FREQ_1, Freq2);
}

void AD9834_SetPSK(float Phase1,float Phase2,unsigned long Freq)//正弦波相位调制
{
    FSYNC_1();
    SCK_1();
    
    RESET_1();
    RESET_1();
    RESET_0();
    AD9834_Write_16Bits(0x0100); //复位
    AD9834_Write_16Bits(0x2228);
	  AD9834_Set_Phase(0,Phase1);
	  AD9834_Set_Phase(1,Phase2);
	  AD9834_Set_Freq(FREQ_0, Freq);
	  AD9834_Set_Freq(FREQ_1, Freq);
}
/****************************************************************
函数名: AD9834_Set_Freq
功能: 设置频率值
参数: freq_chanel -- 要写入的频率寄存器(FREQ_0或FREQ_1)
          freq -- 频率值 (Freq_value(value)=Freq_data(data)*FCLK/2^28)
返回值: 无
*****************************************************************/
void AD9834_Set_Freq(unsigned char freq_chanel, unsigned long freq)
{
    unsigned long FREQREG = (unsigned long)(268435456.0/AD9834_SYSTEM_COLCK*freq);
    
    unsigned int FREQREG_LSB_14BIT = (unsigned int)FREQREG;
    unsigned int FREQREG_MSB_14BIT = (unsigned int)(FREQREG>>14);
    
    if(freq_chanel == FREQ_0)//bit15,14=01
    {  
        FREQREG_LSB_14BIT &= ~(1U<<15);
        FREQREG_LSB_14BIT |= 1<<14;
        FREQREG_MSB_14BIT &= ~(1U<<15);
        FREQREG_MSB_14BIT |= 1<<14;
    }
    else   //bit15,14=10
    {  
        FREQREG_LSB_14BIT &= ~(1<<14);
        FREQREG_LSB_14BIT |= 1U<<15;
        FREQREG_MSB_14BIT &= ~(1<<14);
        FREQREG_MSB_14BIT |= 1U<<15;
    }
    
    AD9834_Write_16Bits(FREQREG_LSB_14BIT);
    AD9834_Write_16Bits(FREQREG_MSB_14BIT);
    
}

/*****************************
函数名: AD9834_Set_Phase
功能: 设置相位值
参数: CH -- 要写入的相位寄存器(FREQ_0或FREQ_1)
          Phase --相位值 0-360，12位数据，4096/360=.....
返回值: 无
*****************************/
void AD9834_Set_Phase(unsigned char CH,float Phase)
{ 
 //   PS = PS * 40.96 / 3.60 + 0.3;
	Phase = Phase * 11.3777778 + 0.3;	
	
	if(CH==0)
	    AD9834_Write_16Bits((unsigned int)Phase + 0xc000);
	else
      AD9834_Write_16Bits((unsigned int)Phase + 0xe000);
}



/*
*********************************************************************************************************
*	函 数 名: AD9833_AmpSet
*	功能说明: 改变输出信号幅度值
*	形    参: 1.amp ：幅度值  0- 255
*	返 回 值: 无
*********************************************************************************************************
*/ 

void AD9834_AmpSet(unsigned char amp)
{
	unsigned char i;
	unsigned int temp;
   
//    if(amp<=10)
//    {
//        amp=10;
//    }
//    
//    if(amp>=250)
//    {
//        amp=250;
//    }
    
	CS_0();
	temp =0x1100|amp;
	for(i=0;i<16;i++)
	{
	    SCK_0();
		AD9834_Delay();
	   if(temp&0x8000)
	   	DAT_1() ;
	   else
	   DAT_0() ;
		temp<<=1;
		 AD9834_Delay();
		 AD9834_Delay();
	    SCK_1();
	    AD9834_Delay();
		 AD9834_Delay();
	}
	
   	CS_1();
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
void AD9834_WaveSeting(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase )
{

		int frequence_LSB,frequence_MSB,Phs_data;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;

		/*********************************计算频率的16进制值***********************************/
		frequence_mid=268435456/75;//适合25M晶振
		//如果时钟频率不为25MHZ，修改该处的频率值，单位MHz ，AD9833最大支持25MHz
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1000000;
		frequence_DATA=frequence_DATA*frequence_mid;
		frequence_hex=frequence_DATA;  //这个frequence_hex的值是32位的一个很大的数字，需要拆分成两个14位进行处理；
		frequence_LSB=frequence_hex; //frequence_hex低16位送给frequence_LSB
		frequence_LSB=frequence_LSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位
		frequence_MSB=frequence_hex>>14; //frequence_hex高16位送给frequence_HSB
		frequence_MSB=frequence_MSB&0x3fff;//去除最高两位，16位数换去掉高位后变成了14位

	  Phs_data = Phs_data * 11.3777778 + 0.3;	
		
		AD9834_Write_16Bits(0x0100); //复位AD9833,即RESET位为1
		AD9834_Write_16Bits(0x2100); //选择数据一次写入，B28位和RESET位为1

		if(Freq_SFR==0)				  //把数据设置到设置频率寄存器0
		{
			Phs_data=Phase|0xC000;	//相位值
		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			 //使用频率寄存器0输出波形
			AD9834_Write_16Bits(frequence_LSB); //L14，选择频率寄存器0的低14位数据输入
			AD9834_Write_16Bits(frequence_MSB); //H14 频率寄存器的高14位数据输入
			AD9834_Write_16Bits(Phs_data);	//设置相位
			//AD9833_Write(0x2000); /**设置FSELECT位为0，芯片进入工作状态,频率寄存器0输出波形**/
	    }
		if(Freq_SFR==1)				//把数据设置到设置频率寄存器1
		{
			Phs_data=Phase|0xE000;	//相位值
			 frequence_LSB=frequence_LSB|0x8000;
			 frequence_MSB=frequence_MSB|0x8000;
			//使用频率寄存器1输出波形
			AD9834_Write_16Bits(frequence_LSB); //L14，选择频率寄存器1的低14位输入
			AD9834_Write_16Bits(frequence_MSB); //H14 频率寄存器1为
			AD9834_Write_16Bits(Phs_data);	//设置相位
			//AD9833_Write(0x2800); /**设置FSELECT位为0，设置FSELECT位为1，即使用频率寄存器1的值，芯片进入工作状态,频率寄存器1输出波形**/
		}

		if(WaveMode==TRI_WAVE) //输出三角波波形
		 	AD9834_Write_16Bits(0x2002); 
		if(WaveMode==SQU_WAVE)	//输出方波波形
			AD9834_Write_16Bits(0x2028); 
		if(WaveMode==SIN_WAVE)	//输出正弦波形
			AD9834_Write_16Bits(0x2000); 

}