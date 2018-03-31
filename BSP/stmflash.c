#include "stmflash.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include <stdio.h>
#include "usart.h"
#include <string.h>

//��ȡָ����ַ�İ���(16λ����)
//faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}
#if STM32_FLASH_WREN	//���ʹ����д   
//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��   
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//��ַ����2.
	}  
} 
//��ָ����ַ��ʼд��ָ�����ȵ�����
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
//pBuffer:����ָ��
//NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //�ֽ�
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //������ַ
	u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
	u16 secremain; //������ʣ���ַ(16λ�ּ���)	   
 	u16 i;    
	u32 offaddr;   //ȥ��0X08000000��ĵ�ַ
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
	FLASH_Unlock();						//����
	offaddr=WriteAddr-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
	secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
	secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С   
	if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
			for(i=0;i<secremain;i++)//����
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������  
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;				//������ַ��1
			secoff=0;				//ƫ��λ��Ϊ0 	 
		   	pBuffer+=secremain;  	//ָ��ƫ��
			WriteAddr+=secremain;	//д��ַƫ��	   
		   	NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//��һ����������д����
			else secremain=NumToWrite;//��һ����������д����
		}	 
	};	
	FLASH_Lock();//����
}
#endif

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
		ReadAddr+=2;//ƫ��2���ֽ�.	
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//WriteAddr:��ʼ��ַ
//WriteData:Ҫд�������
void Test_Write(u32 WriteAddr,u16 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//д��һ���� 
}



u8 acc_offest_buff[32] ;
u8 gyro_offest_buff[32] ;
u8 mag_offest_buff[32];

void Acc_Offest_write()
{
//	u8 offest_buff[32] ;
	
	int16_t temp_acc_x ,temp_acc_y,temp_acc_z;
	
	temp_acc_x =  (sensor.acc.quiet.x );
	temp_acc_y =  (sensor.acc.quiet.y );
	temp_acc_z =  (sensor.acc.quiet.z );
	
	sprintf((char *)acc_offest_buff,"%d,%d,%d.\r\n",temp_acc_x,temp_acc_y,temp_acc_z);

	STMFLASH_Write(ACC_OFFEST_ADDER,(u16*)acc_offest_buff,sizeof(acc_offest_buff));

}


void Gyro_Offest_write()
{
//	u8 offest_buff[32] ;
	
	int16_t temp_gyro_x ,temp_gyro_y,temp_gyro_z;
	
	temp_gyro_x =  (sensor.gyro.quiet.x );
	temp_gyro_y =  (sensor.gyro.quiet.y );
	temp_gyro_z =  (sensor.gyro.quiet.z );

	sprintf((char *)gyro_offest_buff,"%d,%d,%d.\r\n",temp_gyro_x,temp_gyro_y,temp_gyro_z);

	STMFLASH_Write(GYRO_OFFEST_ADDER,(u16*)gyro_offest_buff,sizeof(gyro_offest_buff));

}


void Mag_Offest_write()
{
//	u8 offest_buff[32] ;
	
	int16_t temp_mag_x ,temp_mag_y,temp_mag_z;
	
	temp_mag_x =  (Mag.offset_mx );
	temp_mag_y =  (Mag.offset_mx );
	temp_mag_z =  (Mag.offset_mx );
	
	sprintf((char *)mag_offest_buff,"%d,%d,%d.\r\n",temp_mag_x,temp_mag_y,temp_mag_z);
	
	STMFLASH_Write(MAG_OFFEST_ADDER,(u16*)mag_offest_buff,sizeof(mag_offest_buff));

}

/****************************/
void Acc_Offest_Read()
{
	int16_t temp_acc_x,temp_acc_y,temp_acc_z;

	STMFLASH_Read(ACC_OFFEST_ADDER,(u16*)acc_offest_buff,sizeof(acc_offest_buff));
	
	sscanf((char *)acc_offest_buff,"%d,%d,%d.\r\n",&temp_acc_x,&temp_acc_y,&temp_acc_z);

	sensor.acc.quiet.x =  ((float)temp_acc_x );
	sensor.acc.quiet.y =  ((float)temp_acc_y );
	sensor.acc.quiet.z =  ((float)temp_acc_z );
}


void Gyro_Offest_Read()
{	
	int16_t temp_gyro_x ,temp_gyro_y,temp_gyro_z;
	
	STMFLASH_Read(GYRO_OFFEST_ADDER,(u16*)gyro_offest_buff,sizeof(gyro_offest_buff));

	sscanf((char *)gyro_offest_buff,"%d,%d,%d.\r\n",&temp_gyro_x,&temp_gyro_y,&temp_gyro_z);
	
	sensor.gyro.quiet.x =  ((float)temp_gyro_x);
	sensor.gyro.quiet.y =  ((float)temp_gyro_y);
	sensor.gyro.quiet.z =  ((float)temp_gyro_z);
}

void Mag_Offest_Read()
{	
	int16_t temp_mag_x ,temp_mag_y,temp_mag_z;
	
	STMFLASH_Read(MAG_OFFEST_ADDER,(u16*)mag_offest_buff,sizeof(mag_offest_buff));
	
	sscanf((char *)mag_offest_buff,"%d,%d,%d.\r\n",&temp_mag_x,&temp_mag_y,&temp_mag_z);
	
	Mag.offset_mx =  ((float)temp_mag_x);
	Mag.offset_mx =  ((float)temp_mag_y);
	Mag.offset_mx =  ((float)temp_mag_z);
}







