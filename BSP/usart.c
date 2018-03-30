/******************** (C) COPYRIGHT  Ô´µØ¹¤×÷ÊÒ ********************************
 * ÎÄ¼þÃû  £ºusart.c
 * ÃèÊö    £º½«printfº¯ÊýÖØ¶¨Ïòµ½USART1¡£ÕâÑù¾Í¿ÉÒÔÓÃprintfº¯Êý½«µ¥Æ¬»úµÄÊý¾Ý´ò
 *           Ó¡µ½PCÉÏµÄ³¬¼¶ÖÕ¶Ë»ò´®¿Úµ÷ÊÔÖúÊÖ¡£     
 * ×÷Õß    £ºzhuoyingxingyu
 * ÌÔ±¦    £ºÔ´µØ¹¤×÷ÊÒhttp://vcc-gnd.taobao.com/
 * ÂÛÌ³µØÖ·£º¼«¿ÍÔ°µØ-Ç¶ÈëÊ½¿ª·¢ÂÛÌ³http://vcc-gnd.com/
 * °æ±¾¸üÐÂ: 2015-12-20
 * Ó²¼þÁ¬½Ó: TX->PA9;RX->PA10
 * µ÷ÊÔ·½Ê½£ºJ-Link-OB
**********************************************************************************/	

//Í·ÎÄ¼þ
#include "usart.h"

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
    RCC_AHBPeriphClockCmd(USART_GPIO_RCC, ENABLE);  //Ê¹ÄÜGPIOAµÄÊ±ÖÓ
    RCC_APB2PeriphClockCmd(USART_RCC, ENABLE);//Ê¹ÄÜUSARTµÄÊ±ÖÓ

    GPIO_PinAFConfig(USART_GPIO_PORT, USART_TX_GPIO_PinSource, GPIO_AF_1);//ÅäÖÃPA9³ÉµÚ¶þ¹¦ÄÜÒý½Å	TX
    GPIO_PinAFConfig(USART_GPIO_PORT, USART_RX_GPIO_PinSource, GPIO_AF_1);//ÅäÖÃPA10³ÉµÚ¶þ¹¦ÄÜÒý½Å  RX	

    /*USART1_TX ->PA9  USART1_RX ->PA10*/		
      GPIO_InitStructure.GPIO_Pin = USART_TX|USART_RX;	       //Ñ¡ÖÐ´®¿ÚÄ¬ÈÏÊä³ö¹Ü½Å         
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //¶¨ÒåÊä³ö×î´óËÙÂÊ 
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//¶¨Òå¹Ü½Å9µÄÄ£Ê½  
      GPIO_Init(USART_GPIO_PORT, &GPIO_InitStructure);           //µ÷ÓÃº¯Êý£¬°Ñ½á¹¹Ìå²ÎÊýÊäÈë½øÐÐ³õÊ¼»¯		   

    /*´®¿ÚÍ¨Ñ¶²ÎÊýÉèÖÃ*/
    USART_InitStructure.USART_BaudRate = 115200; //²¨ÌØÂÊ
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Êý¾ÝÎ»8Î»
    USART_InitStructure.USART_StopBits = USART_StopBits_1;	//Í£Ö¹Î»1Î»
    USART_InitStructure.USART_Parity = USART_Parity_No;		//Ð£ÑéÎ» ÎÞ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÁ÷¿ØÖÆ
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//Ê¹ÄÜ½ÓÊÕºÍ·¢ËÍÒý½Å

    USART_Init(USART, &USART_InitStructure);
//	USART_ClearFlag(USART,USART_FLAG_TC);
//	USART_ITConfig(USART, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART, USART_IT_TXE, ENABLE);		
	USART_Cmd(USART, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//´®¿Ú1ÖÐ¶ÏÍ¨µÀ
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;//ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡

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
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //??????
	}
}

  
void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data)  
{  
    USART_SendData(USARTx, Data);  
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}  
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
//	u8 com_data;
//	


//  //½ÓÊÕÖÐ¶Ï
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//??????

//		com_data = USART_ReceiveData(USART1);

	}


} 
