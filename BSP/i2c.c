//ͷ�ļ�
#include "i2c.h"
#include "systick.h"


 /**
  * @file   I2C_Configuration
  * @brief  EEPROM�ܽ�����
  * @param  ��
  * @retval ��
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
  * @brief  �ӳ�ʱ��
  * @param  ��
  * @retval ��
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
  * @brief  ��
  * @param  ��
  * @retval ��
  */
static FunctionalState I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return DISABLE;	/* SDA��Ϊ�͵�ƽ������æ,�˳� */
	SDA_L;
	I2C_delay();
	if(SDA_read) return DISABLE;	/* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
	SDA_L;
	I2C_delay();
	return ENABLE;
}

 /**
  * @file   I2C_Stop
  * @brief  ��
  * @param  ��
  * @retval ��
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
  * @brief  ��
  * @param  ��
  * @retval ��
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
  * @brief  ��
  * @param  ��
  * @retval ��
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
  * @brief  ��
  * @param  ��
  * @retval ����Ϊ:=1��ACK,=0��ACK
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
  * @brief   ���ݴӸ�λ����λ
  * @param    SendByte- SendByte: ���͵�����
  * @retval  ��
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
  * @brief   ���ݴӸ�λ����λ
  * @param    None
  * @retval  I2C���߷��ص�����
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
  * @brief   дһ�ֽ�����
  * @param    
	*        - SendByte: ��д������
	*        - WriteAddress: ��д���ַ
	*        - DeviceAddress: ��������(24c16��SD2403)
  * @retval  ����Ϊ:=1�ɹ�д��,=0ʧ��
  */
FunctionalState I2C_WriteByte(uint8_t SendByte, uint16_t WriteAddress, uint8_t DeviceAddress)
{		
    if(!I2C_Start())return DISABLE;
    I2C_SendByte(((WriteAddress & 0x0700) >>7) | DeviceAddress & 0xFFFE); /*���ø���ʼ��ַ+������ַ */
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte((uint8_t)(WriteAddress & 0x00FF));   /* ���õ���ʼ��ַ */      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
    return ENABLE;
}									 

 /**
  * @file    I2C_ReadByte
  * @brief    ��ȡһ������
  * @param    
	*        - pBuffer: ��Ŷ�������
*          - length: ����������
*          - ReadAddress: ��������ַ
*          - DeviceAddress: ��������(24c16��SD2403)
  * @retval  ����Ϊ:=1�ɹ�д��,=0ʧ��
  */
FunctionalState I2C_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint16_t ReadAddress,  uint8_t DeviceAddress)
{		
    if(!I2C_Start())return DISABLE;
    I2C_SendByte(((ReadAddress & 0x0700) >>7) | DeviceAddress & 0xFFFE); /* ���ø���ʼ��ַ+������ַ */ 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte((uint8_t)(ReadAddress & 0x00FF));   /* ���õ���ʼ��ַ */      
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
* ������:IIC_ADD_write   
* ���� : ���ض��豸id���ض���ַ��д���ֽ� 
* ����  :�豸id���ڲ���ַ������    
* ���  :��    
*/
void IIC_ADD_write(u8 DeviceAddr,u8 address,u8 Bytes)
{
	I2C_Start();
	I2C_SendByte(DeviceAddr);
	I2C_WaitAck();
	I2C_SendByte(address);   //���͵͵�ַ
	I2C_WaitAck(); 	 										  		   
	I2C_SendByte(Bytes);     //�����ֽ�							   
	I2C_WaitAck();  		    	   
	I2C_Stop();//����һ��ֹͣ���� 
	I2C_delay();
}


/************************************************************   
* ������:I2C_ReadByte   
* ���� : ���ض��豸id���ض���ַ��ȡ����
* ����  :�豸id���ڲ���ַ   
* ���  :��ȡ������   
*/
u8 IIC_ADD_read(u8 DeviceAddr,u8 address)
{
	unsigned char temp;
	I2C_Start();
	I2C_SendByte(DeviceAddr);
	I2C_WaitAck();

	I2C_SendByte(address);   //���͵͵�ַ
	I2C_WaitAck();	    
	I2C_Start();  	 	   
	I2C_SendByte(DeviceAddr+1);           //�������ģʽ			   
	I2C_WaitAck();	 
	temp=I2C_ReceiveByte();		   
	I2C_Stop();//����һ��ֹͣ����	    
	return temp;
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
