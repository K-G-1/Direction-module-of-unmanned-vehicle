/*包含头------------------------------------------------------------------*/
#include "i2c.h"
#include "MPU6050.h"
#include "IMU.h"
#include "HMC5883L.h"


///*变量声明----------------------------------------------------------------*/
struct _Mag Mag;
u8 Mag_CALIBRATED=0;

uint8_t HMC5883L_buffer[8];

#define  FILL_NUM  10

int16_t X_BUFF[FILL_NUM],Y_BUFF[FILL_NUM],Z_BUFF[FILL_NUM];

u8 Init_HMC5883L(void)
{
	u8 tempA ,tempB ,tempC;

	IIC_ADD_write(HMC5883L_Addr,0x00,0x70);
	IIC_ADD_write(HMC5883L_Addr,0x01,0x20);
	IIC_ADD_write(HMC5883L_Addr,0x02,0x00);

	tempA= IIC_ADD_read(HMC5883L_Addr, 0x0A);
	tempA= IIC_ADD_read(HMC5883L_Addr, 0x0B);
	tempA= IIC_ADD_read(HMC5883L_Addr, 0x0C);
	
	Mag.HMC5883_maxx = 454;
	Mag.HMC5883_maxy = 744;
	Mag.HMC5883_maxz = 452;
	Mag.HMC5883_minx = -718;
	Mag.HMC5883_miny = -503;
	Mag.HMC5883_minz = -704;
	
	Mag.offset_mx = (Mag.HMC5883_maxx + Mag.HMC5883_minx)/2;
	Mag.offset_my = (Mag.HMC5883_maxy + Mag.HMC5883_miny)/2;
	Mag.offset_mz = (Mag.HMC5883_maxz + Mag.HMC5883_minz)/2;
	
	Mag.mx_scale = 1.0;
	Mag.my_scale = 1.0;
	Mag.mz_scale = 1.0;
    return tempA+tempB+tempC; 		 
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



	Mag.Ori_x = HMC5883L_buffer[0] << 8 | HMC5883L_buffer[1]; //Combine MSB and LSB of X Data output register;
	Mag.Ori_y = HMC5883L_buffer[4] << 8 | HMC5883L_buffer[5]; //Combine MSB and LSB of Y Data output register;
	Mag.Ori_z = HMC5883L_buffer[2] << 8 | HMC5883L_buffer[3]; //Combine MSB and LSB of Z Data output register;
	
}


void Multiple_Read_HMC5883L(void)
{      	
	u8 i;

	static uint8_t filter_cnt=0;
	int32_t temp1=0,temp2=0,temp3=0;
	
	read_HMC5883L();

	X_BUFF[filter_cnt] = Mag.Ori_x; //Combine MSB and LSB of X Data output register;
	Y_BUFF[filter_cnt] = Mag.Ori_y; //Combine MSB and LSB of Y Data output register;
	Z_BUFF[filter_cnt] = Mag.Ori_z; //Combine MSB and LSB of Z Data output register;
	for(i=0;i<FILL_NUM;i++)  //10深度的滑动滤波
	{
		temp1 += X_BUFF[i];
		temp2 += Y_BUFF[i];
		temp3 += Z_BUFF[i];
	}
	Mag.n_x = ( temp1 / FILL_NUM - Mag.offset_mx ) * Mag.mx_scale;
	Mag.n_y = ( temp2 / FILL_NUM - Mag.offset_my ) * Mag.my_scale;
	Mag.n_z = ( temp3 / FILL_NUM - Mag.offset_mz ) * Mag.mz_scale;

	filter_cnt++;
	
	if(filter_cnt==FILL_NUM)	
		filter_cnt=0;

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

	Mag.HMC5883_maxx = -4096;	//将原来的标定值清除
	Mag.HMC5883_maxy = -4096;
	Mag.HMC5883_maxz = -4096;
	Mag.HMC5883_minx = 4096;
	Mag.HMC5883_miny = 4096;
	Mag.HMC5883_minz = 4096;
	Mag.offset_mx = 0;
	Mag.offset_my = 0;
	Mag.offset_mz = 0;
	Mag.mx_scale = 1.0; 
	Mag.my_scale = 1.0;
	Mag.mz_scale = 1.0;	

	Mag.HMC5883_calib_cnt= 0;

	do{

		read_HMC5883L();
					//校正有效的话 采集标定值
		if(Mag.HMC5883_minx>Mag.Ori_x)Mag.HMC5883_minx=(int16_t)Mag.Ori_x;
		if(Mag.HMC5883_miny>Mag.Ori_y)Mag.HMC5883_miny=(int16_t)Mag.Ori_y;
		if(Mag.HMC5883_minz>Mag.Ori_z)Mag.HMC5883_minz=(int16_t)Mag.Ori_z;

		if(Mag.HMC5883_maxx<Mag.Ori_x)Mag.HMC5883_maxx=(int16_t)Mag.Ori_x;
		if(Mag.HMC5883_maxy<Mag.Ori_y)Mag.HMC5883_maxy=(int16_t)Mag.Ori_y;
		if(Mag.HMC5883_maxz<Mag.Ori_z)Mag.HMC5883_maxz=(int16_t)Mag.Ori_z;
		
		Mag.HMC5883_calib_cnt++;
	}while(1);



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
	Mag.offset_mx = (Mag.HMC5883_maxx+Mag.HMC5883_minx)/2;
	Mag.offset_my = (Mag.HMC5883_maxy+Mag.HMC5883_miny)/2;
	Mag.offset_mz = (Mag.HMC5883_maxz+Mag.HMC5883_minz)/2;

}





