#include "./BSP/AD7606/my_ad7606.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_rx;
void MX_SPI4_Init(void)
{
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;  
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /**SPI4 GPIO Configuration
    PE2 ------> SPI4_SCK
    PE5 ------> SPI4_MISO
    PE6 ------> SPI4_MOSI
    */	
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
		
    hspi4.Instance = SPI4;
    hspi4.Init.Mode = SPI_MODE_MASTER;
    hspi4.Init.Direction = SPI_DIRECTION_2LINES;
    hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;   
    hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;        
    hspi4.Init.NSS = SPI_NSS_SOFT;
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  
    hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi4.Init.CRCPolynomial = 0;


    HAL_SPI_Init(&hspi4);
    
    /* 手动使能SPI */
    __HAL_SPI_ENABLE(&hspi4);
}
/**
 * @brief       SPI1读写一个16位数据
 * @param       txdata  : 要发送的数据(2字节)
 * @retval      接收到的数据(2字节)
 */
uint16_t spi1_read_write_16bit(uint16_t txdata)
{
    uint16_t rxdata;
    HAL_SPI_TransmitReceive(&hspi4, (uint8_t *)&txdata, (uint8_t *)&rxdata, 1, 1000);
    return rxdata; /* 返回收到的数据 */
}

/**
 * @brief       AD7606B初始化
 * @param       无
 * @retval      无
 */
void AD7606B_Init(void)
{
	GPIO_InitTypeDef gpio_init_struct;
	__HAL_RCC_GPIOE_CLK_ENABLE();					  	/* 开启GPIOE时钟使能 */
	__HAL_RCC_GPIOI_CLK_ENABLE();					  	/* 开启GPIOI时钟使能 */
	
	gpio_init_struct.Pin =  GPIO_PIN_3 | GPIO_PIN_4;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;      	/* 推挽输出 */
	gpio_init_struct.Pull = GPIO_PULLUP;              	/* 上拉 */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;    	/* 高速 */
	HAL_GPIO_Init(GPIOE, &gpio_init_struct);       	  	/* 初始化引脚 */
	
	// CS引脚配置
  gpio_init_struct.Pin = GPIO_PIN_9;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;  // 明确指定输出模式
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOI, &gpio_init_struct);
	
	gpio_init_struct.Pin = GPIO_PIN_8;
	gpio_init_struct.Mode = GPIO_MODE_INPUT;      		/* 推挽输入 */
	gpio_init_struct.Pull = GPIO_PULLUP;              	/* 上拉 */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;    	/* 高速 */
	HAL_GPIO_Init(GPIOI, &gpio_init_struct);       	  	/* 初始化引脚 */

	MX_SPI4_Init();
	
  ad7606_Reset();        /* 硬件复位复AD7606 */
	AD7606B_Read_Reg(0x02);	// 对寄存器写入操作前需要对任意寄存器执行一次读取操作，使得AD7606B进入寄存器操作模式
	AD7606B_Write_Reg(0x02,0x00);//配置为单线Doutx模式
	
	AD7606B_Write_Reg(0x03,AD7606B_Range_10V);	// CH1-2输入范围设置寄存器，±10V
	AD7606B_Write_Reg(0x04,AD7606B_Range_10V);	// CH3-4输入范围设置寄存器，±10V
	AD7606B_Write_Reg(0x05,AD7606B_Range_10V);	// CH5-6输入范围设置寄存器，±10V
	AD7606B_Write_Reg(0x06,AD7606B_Range_10V);	// CH7-8输入范围设置寄存器，±10V
	AD7606B_Write_Reg(0x08,0x00);				// 无过采样
	
	AD7606B_Write_Reg(0x00,0x00);			// SPI连续输入16个低电平，AD7606B进入ADC模式

//  AD7606B_CNT_H;         /* CONVST脚设置为高电平 */

}

void ad7606_Reset(void)
{
	/* AD7606是高电平复位，要求最小脉宽50ns */
	AD7606B_RST_H() ;
   delay_ms(1);
	
	AD7606B_RST_L();
	 delay_ms(10);
}


/**
 * @brief       AD7606B读寄存器
 * @param       Addr：寄存器地址
 * @retval      寄存器中的数据
 */
uint8_t AD7606B_Read_Reg(uint8_t Addr)
{
	uint16_t RxData = 0;
	uint16_t TxData = 0;
	TxData = 0x40 + Addr;
	TxData = TxData << 8;
	
	AD7606B_CS_L();delay_ms(1);
	RxData = spi1_read_write_16bit(TxData);
	AD7606B_CS_H();
	
	return (uint8_t)RxData;
}

/**
 * @brief AD7606B写寄存器（修正版）
 * @param Addr：寄存器地址，Data：寄存器数据
 * @retval 无
 */
void AD7606B_Write_Reg(uint8_t Addr, uint8_t Data)
{
    uint16_t TxData = ((uint16_t)Addr << 8) | ((uint16_t)Data);
    
    // 确保CS为高开始
    AD7606B_CS_H();
    delay_us(5);
    
    // CS拉低
    AD7606B_CS_L();
    delay_us(5);
    
    // 发送16位数据
    HAL_SPI_TransmitReceive(&hspi4, (uint8_t *)&TxData, (uint8_t *)&TxData, 1, 1000);
    
    // 等待传输完成
    delay_ms(10);
    
    // CS拉高
    AD7606B_CS_H();
    delay_us(10);

}

void AD7606B_Read_AD_Data(int16_t *data)
{
    AD7606B_CS_L();
    
    for(int i = 0; i < 8; i++)
    {
        data[i] = (int16_t)spi1_read_write_16bit(0xFFFF);
    }
    
    AD7606B_CS_H();
}

/**
 * @brief       AD7606B读采集数据
 * @param       data：读取到的8个通道的ADC数据
 * @retval      无
 */
//void AD7606B_Read_AD_Data(int16_t * data)
//{
//	uint16_t TxData[8] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
//   
//	AD7606B_CS_L();
//	HAL_SPI_TransmitReceive(&hspi4, (uint8_t *)TxData, (uint8_t *)data, 8, 1000);
//	AD7606B_CS_H();
//}

void PI8_EXTI_Init(void)
{
 
    __HAL_RCC_GPIOI_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PI8 配置为双边沿中断模式
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;        // 下降触发
    GPIO_InitStruct.Pull = GPIO_PULLUP;                 // 上拉
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    // 配置中断优先级并使能
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}



