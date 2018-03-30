#ifndef __I2C_H
#define __I2C_H			 
#include "stm32f0xx.h"

/* Includes ------------------------------------------------------------------*/


#define EEPROM_I2C_SCL_PIN      					 	GPIO_Pin_4   
#define EEPROM_I2C_SCL_GPIO_PORT   	       				GPIOB  
#define EEPROM_I2C_SCL_GPIO_RCC      			  		RCC_AHBPeriph_GPIOB  
 
#define EEPROM_I2C_SDA_PIN      					 	GPIO_Pin_5   
#define EEPROM_I2C_SDA_GPIO_PORT   	       				GPIOB  
#define EEPROM_I2C_SDA_GPIO_RCC      			  		RCC_AHBPeriph_GPIOB  

/* Private define ------------------------------------------------------------*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_4	 /* GPIO_SetBits(GPIOB , GPIO_Pin_6)   */
#define SCL_L         GPIOB->BRR  = GPIO_Pin_4   /* GPIO_ResetBits(GPIOB , GPIO_Pin_6) */
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_5	 /* GPIO_SetBits(GPIOB , GPIO_Pin_7)   */
#define SDA_L         GPIOB->BRR  = GPIO_Pin_5	 /* GPIO_ResetBits(GPIOB , GPIO_Pin_7) */

#define SCL_read      GPIOB->IDR  & GPIO_Pin_4   /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_6) */
#define SDA_read      GPIOB->IDR  & GPIO_Pin_5	 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_7) */

#define I2C_PageSize  8  /* 24C02Ã¿Ò³8×Ö½Ú */

#define ADDR_24C08		0xA0



/* Private function prototypes -----------------------------------------------*/
void I2C_Configuration(void);
FunctionalState I2C_WriteByte(uint8_t SendByte, uint16_t WriteAddress, uint8_t DeviceAddress);
FunctionalState I2C_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint16_t ReadAddress,  uint8_t DeviceAddress);
void IIC_ADD_write(u8 DeviceAddr,u8 address,u8 Bytes);
u8 IIC_ADD_read(u8 DeviceAddr,u8 address);

#endif 
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
