/*包含头------------------------------------------------------------------*/
#include "i2c.h"
#include "MPU6050.h"
#include "IMU.h"
#include "HMC5883L.h"


///*变量声明----------------------------------------------------------------*/
int Magn_x,Magn_y,Magn_z;    //5883用到
int16_t Magn_Ori_x,Magn_Ori_y,Magn_Ori_z;

u8 Mag_CALIBRATED=0;

uint8_t HMC5883L_buffer[8];



#define  FILL_NUM  10


int16_t X_BUFF[FILL_NUM],Y_BUFF[FILL_NUM],Z_BUFF[FILL_NUM];
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
//磁力计标定值
int  HMC5883_maxx=454,HMC5883_maxy=744,HMC5883_maxz=452,
		 HMC5883_minx=-718,HMC5883_miny=-503,HMC5883_minz=-704;

int16_t offset_mx,offset_my,offset_mz;	
int16_t temp_mx_scale,temp_my_scale,temp_mz_scale;
float mx_scale = 1.0,my_scale = 1.0 ,mz_scale = 1.0;

unsigned char HMC5883_calib=0; //初始化完成标志
u8 HMC5883_cnt=0;

u8 Init_HMC5883L(void)
{
	u8 tempA ,tempB ,tempC;

	IIC_ADD_write(HMC5883L_Addr,0x00,0x70);
	IIC_ADD_write(HMC5883L_Addr,0x01,0x20);
	IIC_ADD_write(HMC5883L_Addr,0x02,0x00);

	tempA= IIC_ADD_read(HMC5883L_Addr, 0x0A);
	tempA= IIC_ADD_read(HMC5883L_Addr, 0x0B);
	tempA= IIC_ADD_read(HMC5883L_Addr, 0x0C);
   return 1; 		 
}



//******************************************************
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//******************************************************
static void read_HMC5883L(void)
{
//		u8 i=0;

	HMC5883L_buffer[0]= IIC_ADD_read(HMC5883L_Addr, 0x03);
	HMC5883L_buffer[1]= IIC_ADD_read(HMC5883L_Addr, 0x04);
	
	HMC5883L_buffer[2]= IIC_ADD_read(HMC5883L_Addr, 0x05);
	HMC5883L_buffer[3]= IIC_ADD_read(HMC5883L_Addr, 0x06);
	
	HMC5883L_buffer[4]= IIC_ADD_read(HMC5883L_Addr, 0x07);
	HMC5883L_buffer[5]= IIC_ADD_read(HMC5883L_Addr, 0x08);

	
//    IIC_Start();                          //起始信号
//    IIC_Send_Byte(HMC5883L_Addr);                   //发送设备地址+写信号
//	  IIC_Wait_Ack();
//    IIC_Send_Byte(0x03);                   //发送存储单元地址，从0x3开始	
//	  IIC_Wait_Ack();
//    IIC_Start();                          //起始信号
//    IIC_Send_Byte(HMC5883L_Addr+1);     //发送设备地址+读信号
//	  IIC_Wait_Ack();
//	  for (i=0; i<6; i++)                   //连续读取6个地址数据，存储中BUF
//    {
//        HMC5883L_buffer[i] = IIC_Read_Byte();          //BUF[0]存储数据
//        if (i == 5)
//           IIC_NAck();                   //最后一个数据需要回NOACK
//        else
//           IIC_Ack();                     //回应ACK
//    }
//    IIC_Stop();                           //停止信号
//		__NOP();
		

	Magn_Ori_x = HMC5883L_buffer[0] << 8 | HMC5883L_buffer[1]; //Combine MSB and LSB of X Data output register;
	Magn_Ori_y = HMC5883L_buffer[4] << 8 | HMC5883L_buffer[5]; //Combine MSB and LSB of Y Data output register;
	Magn_Ori_z = HMC5883L_buffer[2] << 8 | HMC5883L_buffer[3]; //Combine MSB and LSB of Z Data output register;
	
}


void Multiple_Read_HMC5883L(void)
{      	
 	 u8 i;

	 static uint8_t filter_cnt=0;
	 int32_t temp1=0,temp2=0,temp3=0;
	
//    IIC_Start();                          //起始信号
//    IIC_Send_Byte(HMC5883L_Addr);                   //发送设备地址+写信号
//	  IIC_Wait_Ack();
//    IIC_Send_Byte(0x03);                   //发送存储单元地址，从0x3开始	
//	  IIC_Wait_Ack();
//    IIC_Start();                          //起始信号
//    IIC_Send_Byte(HMC5883L_Addr+1);     //发送设备地址+读信号
//	  IIC_Wait_Ack();
//	  for (i=0; i<6; i++)                   //连续读取6个地址数据，存储中BUF
//    {
//        BUF[i] = IIC_Read_Byte();          //BUF[0]存储数据
//        if (i == 5)
//           IIC_NAck();                   //最后一个数据需要回NOACK
//        else
//           IIC_Ack();                     //回应ACK
//    }
//    IIC_Stop();                           //停止信号
//    delay_ms(2);

		read_HMC5883L();
		
		X_BUFF[filter_cnt] = Magn_Ori_x; //Combine MSB and LSB of X Data output register;
	  Y_BUFF[filter_cnt] = Magn_Ori_y; //Combine MSB and LSB of Y Data output register;
	  Z_BUFF[filter_cnt] = Magn_Ori_z; //Combine MSB and LSB of Z Data output register;
	  for(i=0;i<FILL_NUM;i++)  //10深度的滑动滤波
	  {
		   temp1 += X_BUFF[i];
		   temp2 += Y_BUFF[i];
		   temp3 += Z_BUFF[i];
	  }
	  Magn_x = ( temp1 / FILL_NUM - offset_mx ) * mx_scale;
	  Magn_y = ( temp2 / FILL_NUM - offset_my ) * my_scale;
	  Magn_z = ( temp3 / FILL_NUM - offset_mz ) * mz_scale;
		
	  filter_cnt++;
	  if(filter_cnt==FILL_NUM)	filter_cnt=0;
		
		
	if(Mag_CALIBRATED)
	{

		HMC5883L_Start_Calib();

		Mag_CALIBRATED= 0;
		HMC5883L_Save_Calib();
	}


}	   
 
/**************************实现函数********************************************
*函数原型:	  void HMC5883L_Start_Calib(void)
*功　　能:	   进入磁力计标定
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5883L_Start_Calib(void)
{
	HMC5883_calib++;//开始标定
	HMC5883_maxx = -4096;	//将原来的标定值清除
	HMC5883_maxy = -4096;
	HMC5883_maxz = -4096;
	HMC5883_minx = 4096;
	HMC5883_miny = 4096;
	HMC5883_minz = 4096;
	offset_mx = 0;
  offset_my = 0;
  offset_mz = 0;
	mx_scale = 1.0; 
  my_scale = 1.0;
  mz_scale = 1.0;
	
	
//	TIM_Cmd(TIM7, ENABLE);
//	TIM_Cmd(TIM5, DISABLE);
	HMC5883_cnt= 0;
//	TIM_SetCounter(TIM7 ,0);
	do{

		read_HMC5883L();
					//校正有效的话 采集标定值
		if(HMC5883_minx>Magn_Ori_x)HMC5883_minx=(int16_t)Magn_Ori_x;
		if(HMC5883_miny>Magn_Ori_y)HMC5883_miny=(int16_t)Magn_Ori_y;
		if(HMC5883_minz>Magn_Ori_z)HMC5883_minz=(int16_t)Magn_Ori_z;

		if(HMC5883_maxx<Magn_Ori_x)HMC5883_maxx=(int16_t)Magn_Ori_x;
		if(HMC5883_maxy<Magn_Ori_y)HMC5883_maxy=(int16_t)Magn_Ori_y;
		if(HMC5883_maxz<Magn_Ori_z)HMC5883_maxz=(int16_t)Magn_Ori_z;
		
		HMC5883_cnt++;
	}while(1);
//	}while(HMC5883_cnt<= 200);

//	TIM_Cmd(TIM7, DISABLE);
//	TIM_Cmd(TIM5, ENABLE);
	HMC5883_cnt= 0;

}

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_Save_Calib(void)
*功　　能:	  保存磁力计标定值 到Flash
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5883L_Save_Calib(void)
{
	//将磁力计标定值写入 Flash 保存
	offset_mx = (HMC5883_maxx+HMC5883_minx)/2;
  offset_my = (HMC5883_maxy+HMC5883_miny)/2;
  offset_mz = (HMC5883_maxz+HMC5883_minz)/2;
	temp_mx_scale = mx_scale*100;
	temp_my_scale = my_scale*100;
	temp_mz_scale = mz_scale*100;
//	Config.dMx_scale = mx_scale;
//	Config.dMy_scale = my_scale;
//	Config.dMz_scale = mz_scale;

	Mag_CALIBRATED=0; //结束标定

}





