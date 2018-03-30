#include "receivedata.h"
#include "mpu6050.h"
#include "HMC5883L.h"
#include <stdio.h>
#include <string.h>

u8 RxBuff[16];
void ReceiveData(u8 com_data)
{
	
	static u8 input_cnt = 0 ;
	if(com_data == '\r')
	{
		input_cnt = 0;
		commander(RxBuff);
		memset(RxBuff,0, strlen(RxBuff) );
		return ;
	}
	
	
	if(input_cnt == 0&& (com_data >=0x30) )
	{
		RxBuff[0] = com_data;
		input_cnt ++;
	}
	else if(input_cnt == 1)
	{
		RxBuff[1] = com_data;
		input_cnt ++;
	}
	else if(input_cnt == 2)
	{
		RxBuff[2] = com_data;
		input_cnt ++;
	}
	else if(input_cnt == 3)
	{
		RxBuff[3] = com_data;
		input_cnt ++;
	}
	
	
}

void commander(u8 *Buff)
{
	if(!strcmp(Buff,"acc"))
	{
		sensor.acc.CALIBRATE=1;
	}
	else if(!strcmp(Buff,"gyro"))
	{
		sensor.gyro.CALIBRATE=1;
	}
	else if(!strcmp(Buff,"mag"))
	{
		Mag.HMC5883_calib_cnt=1;
	}
	else if(!strcmp(Buff,"over"))
	{
		Mag.HMC5883_calib_cnt=0;
		sensor.gyro.CALIBRATE=0;
		sensor.acc.CALIBRATE=0;
	}
	

}

