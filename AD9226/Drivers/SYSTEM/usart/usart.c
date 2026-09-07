/**
 ****************************************************************************************************					 
 * @file        usart.c
 * @version     V1.1
 * @brief       ���ڳ�ʼ�����룬֧��printf            
 ****************************************************************************************************
 *
 * V1.1
 * �޸�SYS_SUPPORT_OS���ִ���, ����ͷ�ļ��ĳ�:"os.h"
 * ɾ��USART_UX_IRQHandler()�����ĳ�ʱ�������޸�HAL_UART_RxCpltCallback()
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"


/* ���ʹ��os,����������ͷ�ļ�����. */
#if SYS_SUPPORT_OS
#include "os.h"   /* os ʹ�� */
#endif

/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)            /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");  /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE �� stdio.h���涨�� */
FILE __stdout;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while ((USART3->ISR & 0X40) == 0);    /* �ȴ���һ���ַ�������� */
    USART3->TDR = (uint8_t)ch;            /* ��Ҫ���͵��ַ� ch д�뵽TDR�Ĵ��� */
    return ch;
}
#endif

/******************************************************************************************/

#if USART_EN_RX     /* ���ʹ���˽��� */

/* ���ջ���, ���USART_REC_LEN���ֽ�200. */
uint8_t g_usart_rx_buf[USART_REC_LEN];
uint8_t g_usart_rx_buf_2[USART_REC_LEN];
/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
 */
uint16_t g_usart_rx_sta = 0;
uint16_t g_usart_rx_sta_2 = 0;

#endif

uint8_t g_rx_buffer[RXBUFFERSIZE];          /* HAL��ʹ�õĴ��ڽ��ջ��� */
uint8_t g_rx_buffer_2[RXBUFFERSIZE]; 
UART_HandleTypeDef g_uart1_handle;          /* UART��� */
UART_HandleTypeDef huart3;

/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              USART��ʱ��ԴĬ��ѡ��Ϊrcc_pclk1��rcc_pclk2.
 * @retval      ��
 */
void usart_init(uint32_t baudrate)
{
    g_uart1_handle.Instance = USART_UX;                    /* USARTX */
    g_uart1_handle.Init.BaudRate = baudrate;               /* ���ò����� */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;   /* �ֳ�Ϊ8λ���ݸ�ʽ */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;        /* һ��ֹͣλ */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;         /* ����żУ��λ */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* ��Ӳ������ */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;            /* �շ�ģʽ */
    HAL_UART_Init(&g_uart1_handle);                        /* HAL_UART_Init()��ʹ��USART */
    
    /* �ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ��������������� */
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}


void MX_USART3_UART_Init(uint32_t baudrate)
{

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
	
	
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  huart3.Instance = USART3;
  huart3.Init.BaudRate = baudrate;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart3);

  HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);

  HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);

  HAL_UARTEx_DisableFifoMode(&huart3);
	
	    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
		
		HAL_UART_Receive_IT(&huart3, (uint8_t *)g_usart_rx_buf_2, RXBUFFERSIZE);
}



/**
 * @brief       UART�ײ��ʼ������
 * @param       huart: UART�������ָ��
 * @note        �˺����ᱻHAL_UART_Init()����
 *              ���ʱ��ʹ�ܣ��������ã��ж�����
 * @retval      ��
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
  
    if (huart->Instance == USART_UX)                                /* ����Ǵ���X�����д���X MSP��ʼ�� */
    {
        USART_UX_CLK_ENABLE();                                      /* USARTX ʱ��ʹ�� */
        USART_TX_GPIO_CLK_ENABLE();                                 /* ����TX����ʱ��ʹ�� */
        USART_RX_GPIO_CLK_ENABLE();                                 /* ����RX����ʱ��ʹ�� */

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;                   /* USARTX TX���� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* ����������� */
        gpio_init_struct.Pull = GPIO_PULLUP;                        /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;              /* ����ΪUSARTX */
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);       /* ��ʼ���������� */

        gpio_init_struct.Pin = USART_RX_GPIO_PIN;                   /* USARTX RX���� */
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;              /* ����ΪUSARTX */
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);       /* ��ʼ���������� */

#if USART_EN_RX                                                     /* ���ʹ���˽��� */
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                          /* ʹ��USARTX�ж�ͨ�� */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);                  /* ��ռ���ȼ�3�������ȼ�3 */
#endif
    }
}

#if USART_EN_RX     /* ���ʹ���˽��� */

/**
 * @brief       Rx����ص�����
 * @param       huart: UART�������ָ��
 * @retval      ��
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART_UX)                          /* ����Ǵ���X */
    {
        if ((g_usart_rx_sta & 0x8000) == 0)                   /* ����δ��� */
        {
            if (g_usart_rx_sta & 0x4000)                      /* ���յ���0x0d? */
            {
                if (g_rx_buffer[0] != 0x0a)                   /* ���յ���0x0a? (�����Ƚ��յ�0x0d,�ż��0x0a) */
                {
                    g_usart_rx_sta = 0;                       /* ���մ���,���¿�ʼ */
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;                 /* �յ���0x0a,��ǽ�������� */
                }
            }
            else                                              /* ��û�յ�0X0d */
            {
                if (g_rx_buffer[0] == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;                 /* ��ǽ��յ���0x0d */
                }
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0];   /* �洢���ݵ� g_usart_rx_buf */
                    g_usart_rx_sta++;
                  
                    if (g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;                   /* �����������,���¿�ʼ���� */
                    }
                }
            }
        }
        
        HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);	
    }
    if (huart->Instance == USART3)                          /* USART3: 'd'=header, 'f'=terminator */
    {
        if ((g_usart_rx_sta_2 & 0x8000) == 0)                   /* frame not complete yet */
        {
            uint8_t ch = g_rx_buffer_2[0];

            if (ch == 'f' && (g_usart_rx_sta_2 & 0x4000))       /* 'f' after header → frame done */
            {
                g_usart_rx_sta_2 |= 0x8000;
            }
            else if (ch == 'd')                                  /* 'd' → new frame header */
            {
                g_usart_rx_sta_2 = 0;
                g_usart_rx_buf_2[0] = 'd';
                g_usart_rx_sta_2 = 0x4000 | 1;                  /* bit14=header, len=1 */
            }
            else if (g_usart_rx_sta_2 & 0x4000)                  /* inside frame (after header) */
            {
                uint16_t len = g_usart_rx_sta_2 & 0x3FFF;
                g_usart_rx_buf_2[len] = ch;
                len++;
                g_usart_rx_sta_2 = (g_usart_rx_sta_2 & 0xC000) | len;
                if (len >= (USART_REC_LEN - 1))
                    g_usart_rx_sta_2 = 0;                        /* overflow, reset */
            }
            /* else: ignore bytes before 'd' */
        }

        HAL_UART_Receive_IT(&huart3, (uint8_t *)g_rx_buffer_2, RXBUFFERSIZE);
    }		
		
		
}

//ע����ǣ���ͷ��d����β������f������������ѭ��










/**
 * @brief       ����X�жϷ�����
 * @param       ��
 * @retval      ��
 */
void USART_UX_IRQHandler(void)
{
#if SYS_SUPPORT_OS
    OSIntEnter();
#endif

    HAL_UART_IRQHandler(&g_uart1_handle);

#if SYS_SUPPORT_OS
    OSIntExit();
#endif
}

/**
 * @brief       USART3 interrupt handler (PB10/PB11)
 */
void USART3_IRQHandler(void)
{
#if SYS_SUPPORT_OS
    OSIntEnter();
#endif

    HAL_UART_IRQHandler(&huart3);

#if SYS_SUPPORT_OS
    OSIntExit();
#endif
}

#endif


 

 



 
