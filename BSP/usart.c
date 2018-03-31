//Í·ÎÄ¼þ
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
  * @brief  USART1 GPIO ÅäÖÃ,¹¤×÷Ä£Ê½ÅäÖÃ¡£9600-8-N-1
  * @param  ÎÞ
  * @retval ÎÞ
  */
void USART1_Config(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;  //¶¨Òå´®¿Ú³õÊ¼»¯½á¹¹Ìå
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  //Ê¹ÄÜGPIOAµÄÊ±ÖÓ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//Ê¹ÄÜUSARTµÄÊ±ÖÓ

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//ÅäÖÃPA9³ÉµÚ¶þ¹¦ÄÜÒý½Å	TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//ÅäÖÃPA10³ÉµÚ¶þ¹¦ÄÜÒý½Å  RX	

    /*USART1_TX ->PA9  USART1_RX ->PA10*/		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	       //Ñ¡ÖÐ´®¿ÚÄ¬ÈÏÊä³ö¹Ü½Å         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //¶¨ÒåÊä³ö×î´óËÙÂÊ 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//¶¨Òå¹Ü½Å9µÄÄ£Ê½  
	GPIO_Init(GPIOA, &GPIO_InitStructure);           //µ÷ÓÃº¯Êý£¬°Ñ½á¹¹Ìå²ÎÊýÊäÈë½øÐÐ³õÊ¼»¯		   

    /*´®¿ÚÍ¨Ñ¶²ÎÊýÉèÖÃ*/
    USART_InitStructure.USART_BaudRate = 115200; //²¨ÌØÂÊ
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Êý¾ÝÎ»8Î»
    USART_InitStructure.USART_StopBits = USART_StopBits_1;	//Í£Ö¹Î»1Î»
    USART_InitStructure.USART_Parity = USART_Parity_No;		//Ð£ÑéÎ» ÎÞ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÁ÷¿ØÖÆ
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//Ê¹ÄÜ½ÓÊÕºÍ·¢ËÍÒý½Å

    USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART1, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//´®¿Ú1ÖÐ¶ÏÍ¨µÀ
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x02;//ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡

}


 /**
  * @file   USART2_Config
  * @brief  USART2 GPIO ÅäÖÃ,¹¤×÷Ä£Ê½ÅäÖÃ¡£9600-8-N-1
  * @param  ÎÞ
  * @retval ÎÞ
  */
void USART2_Config(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;	
	USART_InitTypeDef USART_InitStructure;  //¶¨Òå´®¿Ú³õÊ¼»¯½á¹¹Ìå
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_AHBPeriphClockCmd(USART_GPIO_RCC, ENABLE);  //Ê¹ÄÜGPIOAµÄÊ±ÖÓ
	RCC_APB1PeriphClockCmd(USART_RCC, ENABLE);//Ê¹ÄÜUSARTµÄÊ±ÖÓ

	GPIO_PinAFConfig(USART_GPIO_PORT, USART_TX_GPIO_PinSource, GPIO_AF_1);//ÅäÖÃPA9³ÉµÚ¶þ¹¦ÄÜÒý½Å	TX
	GPIO_PinAFConfig(USART_GPIO_PORT, USART_RX_GPIO_PinSource, GPIO_AF_1);//ÅäÖÃPA10³ÉµÚ¶þ¹¦ÄÜÒý½Å  RX	

    /*USART1_TX ->PA2 USART1_RX ->PA3*/			
    GPIO_InitStructure.GPIO_Pin = USART_TX|USART_RX;	       //Ñ¡ÖÐ´®¿ÚÄ¬ÈÏÊä³ö¹Ü½Å         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //¶¨ÒåÊä³ö×î´óËÙÂÊ 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//¶¨Òå¹Ü½Å9µÄÄ£Ê½  
    GPIO_Init(USART_GPIO_PORT, &GPIO_InitStructure);           //µ÷ÓÃº¯Êý£¬°Ñ½á¹¹Ìå²ÎÊýÊäÈë½øÐÐ³õÊ¼»¯		   
    /*´®¿ÚÍ¨Ñ¶²ÎÊýÉèÖÃ*/
    USART_InitStructure.USART_BaudRate = BaudRate; //²¨ÌØÂÊ
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Êý¾ÝÎ»8Î»
    USART_InitStructure.USART_StopBits = USART_StopBits_1;	//Í£Ö¹Î»1Î»
    USART_InitStructure.USART_Parity = USART_Parity_No;		//Ð£ÑéÎ» ÎÞ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÁ÷¿ØÖÆ
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//Ê¹ÄÜ½ÓÊÕºÍ·¢ËÍÒý½Å



                                /* USART2µÄNVICÖÐ¶ÏÅäÖÃ */
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


void USART1_IRQHandler(void)                	//´®¿Ú1ÖÐ¶Ï·þÎñ³ÌÐò
{
	u8 com_data;

//  //½ÓÊÕÖÐ¶Ï
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		com_data = USART_ReceiveData(USART1);
//		ReceiveData(com_data);
		USART_SendData(USART1, com_data);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
} 

void USART2_IRQHandler(void)                	//´®¿Ú1ÖÐ¶Ï·þÎñ³ÌÐò
{
	u8 com_data;

//  //½ÓÊÕÖÐ¶Ï
	if( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET )
	{
		com_data = USART_ReceiveData(USART2);
		ReceiveData(com_data);
		USART_SendData(USART2, com_data);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
} 
