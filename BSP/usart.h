#ifndef __USART_H
#define __USART_H

#include "stm32f0xx.h"
#include <stdio.h>

#define BaudRate	 115200

#define USART              	 								USART2
#define USART_RCC         	 								RCC_APB1Periph_USART2

#define USART_GPIO_RCC    									RCC_AHBPeriph_GPIOA
#define USART_TX_GPIO_PinSource		        	GPIO_PinSource2
#define USART_RX_GPIO_PinSource		        	GPIO_PinSource3	
#define USART_TX		       		 							GPIO_Pin_2	// out
#define USART_RX		       		 							GPIO_Pin_3	// in 
#define USART_GPIO_PORT    									GPIOA   


void USART1_Config(void);
void USART2_Config(void);
void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data)  ;
void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str) ;
#endif
