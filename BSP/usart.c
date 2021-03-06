//头文件
#include "usart.h"
#include "receivedata.h"


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


 /**
  * @file   USART1_Config
  * @brief  USART1 GPIO 配置,工作模式配置。9600-8-N-1
  * @param  无
  * @retval 无
  */
void USART1_Config(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;  //定义串口初始化结构体
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  //使能GPIOA的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//使能USART的时钟

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//配置PA9成第二功能引脚	TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//配置PA10成第二功能引脚  RX	

    /*USART1_TX ->PA9  USART1_RX ->PA10*/		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	       //选中串口默认输出管脚         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //定义输出最大速率 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//定义管脚9的模式  
	GPIO_Init(GPIOA, &GPIO_InitStructure);           //调用函数，把结构体参数输入进行初始化		   

    /*串口通讯参数设置*/
    USART_InitStructure.USART_BaudRate = 115200; //波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;		//校验位 无
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//使能接收和发送引脚

    USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART1, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x02;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器�

}


 /**
  * @file   USART2_Config
  * @brief  USART2 GPIO 配置,工作模式配置。9600-8-N-1
  * @param  无
  * @retval 无
  */
void USART2_Config(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;	
	USART_InitTypeDef USART_InitStructure;  //定义串口初始化结构体
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_AHBPeriphClockCmd(USART_GPIO_RCC, ENABLE);  //使能GPIOA的时钟
	RCC_APB1PeriphClockCmd(USART_RCC, ENABLE);//使能USART的时钟

	GPIO_PinAFConfig(USART_GPIO_PORT, USART_TX_GPIO_PinSource, GPIO_AF_1);//配置PA9成第二功能引脚	TX
	GPIO_PinAFConfig(USART_GPIO_PORT, USART_RX_GPIO_PinSource, GPIO_AF_1);//配置PA10成第二功能引脚  RX	

    /*USART1_TX ->PA2 USART1_RX ->PA3*/			
    GPIO_InitStructure.GPIO_Pin = USART_TX|USART_RX;	       //选中串口默认输出管脚         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //定义输出最大速率 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//定义管脚9的模式  
    GPIO_Init(USART_GPIO_PORT, &GPIO_InitStructure);           //调用函数，把结构体参数输入进行初始化		   
    /*串口通讯参数设置*/
    USART_InitStructure.USART_BaudRate = BaudRate; //波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;		//校验位 无
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//使能接收和发送引脚



                                /* USART2的NVIC中断配置 */
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
	
	
	USART_Init(USART, &USART_InitStructure);
    USART_ClearFlag(USART,USART_FLAG_TC);
	USART_ClearFlag(USART,USART_IT_RXNE);
    USART_ITConfig(USART, USART_IT_RXNE, ENABLE);		
    USART_Cmd(USART, ENABLE);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART, (uint8_t) ch);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART, USART_FLAG_TC) == RESET)
  {}
  return ch;
}

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 


void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
{
	  u8 i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}

	if(!(USART1->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
	}
}

  
void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data)  
{  
    USART_SendData(USARTx, Data);  
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){}  
}  
void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str)    
{    
    while (0 != *str)    
    {    
        UART_PutChar(USARTx, *str);    
        str++;    
    }    
}   


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 com_data;

//  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		com_data = USART_ReceiveData(USART1);
//		ReceiveData(com_data);
		USART_SendData(USART1, com_data);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
} 

void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	u8 com_data;

//  //接收中断
	if( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET )
	{
		com_data = USART_ReceiveData(USART2);
		ReceiveData(com_data);
		USART_SendData(USART2, com_data);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
} 
