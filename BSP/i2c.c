//头文件
#include "i2c.h"
#include "systick.h"


 /**
  * @file   I2C_Configuration
  * @brief  EEPROM管脚配置
  * @param  无
  * @retval 无
  */
void I2C_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	/* Configure I2C2 pins: PB6->SCL and PB7->SDA */
	RCC_AHBPeriphClockCmd(EEPROM_I2C_SCL_GPIO_RCC|EEPROM_I2C_SDA_GPIO_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = EEPROM_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_Init(EEPROM_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = EEPROM_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  
	GPIO_Init(EEPROM_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
	SDA_H;
	SCL_H;
}

 /**
  * @file   I2C_delay
  * @brief  延迟时间
  * @param  无
  * @retval 无
  */
static void I2C_delay(void)
{	
   uint8_t i=50; 
   while(i) 
   { 
     i--; 
   } 
}

 /**
  * @file   I2C_Start
  * @brief  无
  * @param  无
  * @retval 无
  */
static FunctionalState I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return DISABLE;	/* SDA线为低电平则总线忙,退出 */
	SDA_L;
	I2C_delay();
	if(SDA_read) return DISABLE;	/* SDA线为高电平则总线出错,退出 */
	SDA_L;
	I2C_delay();
	return ENABLE;
}

 /**
  * @file   I2C_Stop
  * @brief  无
  * @param  无
  * @retval 无
  */
static void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

 /**
  * @file   I2C_Ack
  * @brief  无
  * @param  无
  * @retval 无
  */
static void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

 /**
  * @file   I2C_NoAck
  * @brief  无
  * @param  无
  * @retval 无
  */
static void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

 /**
  * @file   I2C_WaitAck
  * @brief  无
  * @param  无
  * @retval 返回为:=1有ACK,=0无ACK
  */
static FunctionalState I2C_WaitAck(void) 	
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
      return DISABLE;
	}
	SCL_L;
	return ENABLE;
}

 /**
  * @file    I2C_SendByte
  * @brief   数据从高位到低位
  * @param    SendByte- SendByte: 发送的数据
  * @retval  无
  */
static void I2C_SendByte(uint8_t SendByte) 
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}

 /**
  * @file    I2C_ReceiveByte
  * @brief   数据从高位到低位
  * @param    None
  * @retval  I2C总线返回的数据
  */
static uint8_t I2C_ReceiveByte(void)  
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
}
 
 /**
  * @file    I2C_WriteByte
  * @brief   写一字节数据
  * @param    
	*        - SendByte: 待写入数据
	*        - WriteAddress: 待写入地址
	*        - DeviceAddress: 器件类型(24c16或SD2403)
  * @retval  返回为:=1成功写入,=0失败
  */
FunctionalState I2C_WriteByte(uint8_t SendByte, uint16_t WriteAddress, uint8_t DeviceAddress)
{		
    if(!I2C_Start())return DISABLE;
    I2C_SendByte(((WriteAddress & 0x0700) >>7) | DeviceAddress & 0xFFFE); /*设置高起始地址+器件地址 */
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte((uint8_t)(WriteAddress & 0x00FF));   /* 设置低起始地址 */      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
    return ENABLE;
}									 

 /**
  * @file    I2C_ReadByte
  * @brief    读取一串数据
  * @param    
	*        - pBuffer: 存放读出数据
*          - length: 待读出长度
*          - ReadAddress: 待读出地址
*          - DeviceAddress: 器件类型(24c16或SD2403)
  * @retval  返回为:=1成功写入,=0失败
  */
FunctionalState I2C_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint16_t ReadAddress,  uint8_t DeviceAddress)
{		
    if(!I2C_Start())return DISABLE;
    I2C_SendByte(((ReadAddress & 0x0700) >>7) | DeviceAddress & 0xFFFE); /* 设置高起始地址+器件地址 */ 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte((uint8_t)(ReadAddress & 0x00FF));   /* 设置低起始地址 */      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(((ReadAddress & 0x0700) >>7) | DeviceAddress | 0x0001);
    I2C_WaitAck();
    while(length)
    {
      *pBuffer = I2C_ReceiveByte();
      if(length == 1)I2C_NoAck();
      else I2C_Ack(); 
      pBuffer++;
      length--;
    }
    I2C_Stop();
    return ENABLE;
}


/************************************************************   
* 函数名:IIC_ADD_write   
* 描述 : 向特定设备id的特定地址，写入字节 
* 输入  :设备id，内部地址，数据    
* 输出  :无    
*/
void IIC_ADD_write(u8 DeviceAddr,u8 address,u8 Bytes)
{
	I2C_Start();
	I2C_SendByte(DeviceAddr);
	I2C_WaitAck();
	I2C_SendByte(address);   //发送低地址
	I2C_WaitAck(); 	 										  		   
	I2C_SendByte(Bytes);     //发送字节							   
	I2C_WaitAck();  		    	   
	I2C_Stop();//产生一个停止条件 
	I2C_delay();
}


/************************************************************   
* 函数名:I2C_ReadByte   
* 描述 : 从特定设备id的特定地址读取内容
* 输入  :设备id，内部地址   
* 输出  :读取的内容   
*/
u8 IIC_ADD_read(u8 DeviceAddr,u8 address)
{
	unsigned char temp;
	I2C_Start();
	I2C_SendByte(DeviceAddr);
	I2C_WaitAck();

	I2C_SendByte(address);   //发送低地址
	I2C_WaitAck();	    
	I2C_Start();  	 	   
	I2C_SendByte(DeviceAddr+1);           //进入接收模式			   
	I2C_WaitAck();	 
	temp=I2C_ReceiveByte();		   
	I2C_Stop();//产生一个停止条件	    
	return temp;
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
