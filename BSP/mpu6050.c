#include "i2c.h"
#include "MPU6050.h"
#include "IMU.h"
#include "HMC5883L.h"
	

u8		 mpu6050_buffer[15];
u16 Gyro_x;
struct _sensor sensor;

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};

////////////////////////////////////////////////////////////////////////////////// 	


//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res;


	IIC_ADD_write(MPU_ADDR,MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	
	IIC_ADD_write(MPU_ADDR,0x19, 0x07);    //陀螺仪采样率
//	IIC_ADD_write(MPU_ADDR,MPU_CFG_REG, 0x04);        //25Hz 
	
	IIC_ADD_write(MPU_ADDR,MPU_INTBP_CFG_REG, 0x42);   //使能旁路I2C
	IIC_ADD_write(MPU_ADDR,MPU_USER_CTRL_REG, 0x40);     //使能旁路I2C
	IIC_ADD_write(MPU_ADDR,MPU_CFG_REG, 0x02);     //低通滤波
	IIC_ADD_write(MPU_ADDR,MPU_GYRO_CFG_REG, 0x10);      //+-1000
	IIC_ADD_write(MPU_ADDR,MPU_ACCEL_CFG_REG, 0x09);
	
	
	res=IIC_ADD_read(MPU_ADDR,MPU_DEVICE_ID_REG);
	if(res==0x68)//器件ID正确
	{
		return 0;
 	}else 
	{
		return 1;
	}
}




///*********************************************
////   u8 MPU_READ_LEN(u8 addr,u8 reg,u8 len,u8 *buf)
////   器件地址，寄存器，长度，数据
////	 输出BUF【】
//*********************************************/
//u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
//{
//	IIC_Start(); 
//	IIC_Send_Byte(addr);//发送器件地址+写命令	
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);	//写寄存器地址
//	IIC_Wait_Ack();		//等待应答
//	IIC_Start();
//	IIC_Send_Byte(addr);//发送器件地址+读命令	
//	IIC_Wait_Ack();		//等待应答 
//	while(len)
//	{
//		if(len==1)
//		{
//			*buf=IIC_Read_Byte();//读数据,发送nACK 
//			IIC_NAck();
//		}
//		else 
//		{
//			*buf=IIC_Read_Byte();		//读数据,发送ACK
//			IIC_Ack();
//		}			
//		len--;
//		buf++; 
//	}    
//    IIC_Stop();	//产生一个停止条件 
//	return 0;	
//}
////得到温度值
////返回值:温度值(扩大了100倍)
//short MPU_Get_Temperature(void)
//{
//    u8 buf[2]; 
//    short raw;
//	  float temp;

//	  MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
//    raw=((u16)buf[0]<<8)|buf[1];  
//    temp=36.53+((double)raw)/340;  
//    return temp*100;;
//}



////得到陀螺仪值(原始值)
////gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
////返回值:0,成功
////    其他,错误代码
//u8 MPU_Get_Gyroscope(void)
//{
//   
//	u8 buf[6],res; 
//	u32 gx,gy,gz;
//	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
//	if(res==0)
//	{
//		gx=((u16)buf[0]<<8)|buf[1];  
//		gy=((u16)buf[2]<<8)|buf[3];  
//		gz=((u16)buf[4]<<8)|buf[5];
//	} 	

//    return res;;
//	
//}
////得到加速度值(原始值)
////gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
////返回值:0,成功
////    其他,错误代码
//u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
//{
//    u8 buf[6],res;  
//	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
//	if(res==0)
//	{
//		*ax=((u16)buf[0]<<8)|buf[1];  
//		*ay=((u16)buf[2]<<8)|buf[3];  
//		*az=((u16)buf[4]<<8)|buf[5];
//	} 	

//    return res;;
//}


/*********************************************
//   void MPU_Read(void)
//   
//	 
*********************************************/

void MPU_Read(void)
{

	mpu6050_buffer[0]=IIC_ADD_read(MPU_ADDR, 0x3B);
	mpu6050_buffer[1]=IIC_ADD_read(MPU_ADDR, 0x3C);
	mpu6050_buffer[2]=IIC_ADD_read(MPU_ADDR, 0x3D);
	mpu6050_buffer[3]=IIC_ADD_read(MPU_ADDR, 0x3E);
	mpu6050_buffer[4]=IIC_ADD_read(MPU_ADDR, 0x3F);
	mpu6050_buffer[5]=IIC_ADD_read(MPU_ADDR, 0x40);
	
	mpu6050_buffer[6]=IIC_ADD_read(MPU_ADDR, 0x41);
	mpu6050_buffer[7]=IIC_ADD_read(MPU_ADDR, 0x42);
	
	mpu6050_buffer[8]=IIC_ADD_read(MPU_ADDR, 0x43);
	mpu6050_buffer[9]=IIC_ADD_read(MPU_ADDR, 0x44);
	mpu6050_buffer[10]=IIC_ADD_read(MPU_ADDR, 0x45);
	mpu6050_buffer[11]=IIC_ADD_read(MPU_ADDR, 0x46);
	mpu6050_buffer[12]=IIC_ADD_read(MPU_ADDR, 0x47);
	mpu6050_buffer[13]=IIC_ADD_read(MPU_ADDR, 0x48);
	
//
}


/*********************************************
//   void MPU_Updata(void)
//   
//	 
*********************************************/
void MPU_Updata(void)
{
	u16 Temperature;
	MPU_Read();
	

	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x;
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;

	sensor.acc.radian.x = (float )sensor.acc.origin.x ;
	sensor.acc.radian.y = (float )sensor.acc.origin.y ;
	sensor.acc.radian.z = (float )sensor.acc.origin.z ;
	
	
	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
  
	Temperature=((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]);
	Temperature = (36.53+((double)Temperature)/340)*10;
	
//	sensor.gyro.sand.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x )* Gyro_Gr ;
//	sensor.gyro.sand.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y )* Gyro_Gr ;
//	sensor.gyro.sand.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z )* Gyro_Gr ;

//	sensor.gyro.radian1.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x )* Gyro_Gr;
//	sensor.gyro.radian1.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y )* Gyro_Gr;
//	sensor.gyro.radian1.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z )* Gyro_Gr;
	
//	sensor.gyro.radian.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x ) * Gyro_Gr ;
//	sensor.gyro.radian.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y )* Gyro_Gr ;
//	sensor.gyro.radian.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z )* Gyro_Gr ;
//	
	sensor.gyro.radian.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x )  ;
	sensor.gyro.radian.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y ) ;
	sensor.gyro.radian.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z ) ;

//	if(sensor.acc.CALIBRATE==1)
//	{
//	  MPU_Acc_Offset();

//		sensor.acc.CALIBRATE=0;
//	}

//	if(sensor.gyro.CALIBRATE==1)
//	{
//		MPU_Gyro_Offset();	

//		sensor.gyro.CALIBRATE=0;
//	}
}



/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/
#define KALMAN_Q        0.05
#define KALMAN_R        5.0000

static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }

static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }



/*====================================================================================================*
**函数 : LPF_1st
**功能 : 一阶低通滤波
**输入 :  
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}




void Prepare_Data(void)
{
	static float x,y,z;
//	static float File_Buf[3][20];
//	static int num = 0;
//	int i=0;
// 	float  sum1=0,sum2=0,sum3=0;

	
	MPU_Updata();
    Multiple_Read_HMC5883L();
//	SampleANDExchange();                 //气压计数值
	
//	
	sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,KALMAN_Q,KALMAN_R);  // ACC X轴卡尔曼滤波
	sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,KALMAN_Q,KALMAN_R);  // ACC Y轴卡尔曼滤波
	sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,KALMAN_Q,KALMAN_R);  // ACC Z轴卡尔曼滤波
	
//  sensor.gyro.radian1.x = KalmanFilter_x(sensor.gyro.radian.x,KALMAN_Q,KALMAN_R);  // ACC X轴卡尔曼滤波
//	sensor.gyro.radian1.y = KalmanFilter_y(sensor.gyro.radian.y,KALMAN_Q,KALMAN_R);  // ACC Y轴卡尔曼滤波
//	sensor.gyro.radian1.z = KalmanFilter_z(sensor.gyro.radian.z,KALMAN_Q,KALMAN_R);  // ACC Z轴卡尔曼滤波
	
	
//	sensor.gyro.radian1.x = IIR_I_Filter(sensor.gyro.radian.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	sensor.gyro.radian1.y = IIR_I_Filter(sensor.gyro.radian.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	sensor.gyro.radian1.z = IIR_I_Filter(sensor.gyro.radian.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);	
//	sensor.acc.averag.x = IIR_I_Filter(sensor.acc.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	sensor.acc.averag.y = IIR_I_Filter(sensor.acc.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	sensor.acc.averag.z = IIR_I_Filter(sensor.acc.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);	
//	

// 陀螺仪一阶低通滤波
 	sensor.gyro.radian1.x = LPF_1st(x,sensor.gyro.radian.x ,0.386f);	x = sensor.gyro.radian1.x;    //（0.2 - 0.4）
 	sensor.gyro.radian1.y = LPF_1st(y,sensor.gyro.radian.y ,0.386f);	y = sensor.gyro.radian1.y;
	sensor.gyro.radian1.z = LPF_1st(z,sensor.gyro.radian.z  ,0.386f);	z = sensor.gyro.radian1.z;//
	sensor.gyro.averag.x = sensor.gyro.radian1.x *Gyro_Gr;
	sensor.gyro.averag.y = sensor.gyro.radian1.y *Gyro_Gr;
	sensor.gyro.averag.z = sensor.gyro.radian1.z *Gyro_Gr;
//	sensor.acc.averag.x = LPF_1st(x1,sensor.acc.origin.x ,0.17f);	x1 = sensor.acc.averag.x;
// 	sensor.acc.averag.y = LPF_1st(y1,sensor.acc.origin.y ,0.17f);	y1= sensor.acc.averag.y;
//	sensor.acc.averag.z = LPF_1st(z1,sensor.acc.origin.z  ,0.17f);	z1 = sensor.acc.averag.z;//
//	
//	File_Buf[0][num] = sensor.acc.origin.x;
//	File_Buf[1][num] = sensor.acc.origin.y;
//	File_Buf[2][num] = sensor.acc.origin.z;

////////	File_Buf[3][num] = sensor.gyro.radian.x;
////////	File_Buf[4][num] = sensor.gyro.radian.y;
////////	File_Buf[5][num] = sensor.gyro.radian.z;

//	
//	for(i=0;i<20;i++)
//	{
//     sum1 += File_Buf[0][i];
//		 sum2 += File_Buf[1][i];
//		 sum3 += File_Buf[2][i];
////		 sum4 += File_Buf[3][i];
////		 sum5 += File_Buf[4][i];
////		 sum6 += File_Buf[5][i];
//  }

////////	sensor.gyro.radian1.x = sum1 / 10;
////////	sensor.gyro.radian1.y = sum2 / 10;
////////	sensor.gyro.radian1.z = sum3 / 10;
//////	
//////	
////	
//	sensor.acc.averag.x = sum1 / 20.0f;
//	sensor.acc.averag.y = sum2 / 20.0f;
//	sensor.acc.averag.z = sum3 / 20.0f;
//	
//	num = (num + 1) % 20;
////	

	

}	



/*
 * 函数名：MPU_Acc_Offset
 * 描述  ：MPU数据校准
 * 输入  ：校准次数
 * 输出  ：无
 */ 
void MPU_Acc_Offset(void)
{
	u16 cnt=200;
  sensor.acc.temp.x=0;
	sensor.acc.temp.y=0;
	sensor.acc.temp.z=0;

	 sensor.gyro.averag.x=0;    //零点偏移清零
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt--)       //循环采集2000次   求平均
	 {
		  MPU_Read();

		 
			sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
			sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
			sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

		 
		  sensor.acc.temp.x+= sensor.acc.origin.x;
			sensor.acc.temp.y+= sensor.acc.origin.y;
			sensor.acc.temp.z+= sensor.acc.origin.z;
   }
	 cnt=200;

	 
	 sensor.acc.quiet.x=(sensor.acc.temp.x/cnt);
	 sensor.acc.quiet.y=sensor.acc.temp.y/cnt;
	 sensor.acc.quiet.z=sensor.acc.temp.z/cnt;
	 
//	AT24cxx_save_Acc_Gyro_offest();

	 
}	

void MPU_Gyro_Offset(void)
{
	u16 cnt=200;
	
	sensor.gyro.temp.x=0;
	sensor.gyro.temp.y=0;
	sensor.gyro.temp.z=0;
	
	sensor.acc.averag.x=0;    //零点偏移清零
	sensor.acc.averag.y=0;  
	sensor.acc.averag.z=0;
	
	 while(cnt--)       //循环采集2000次   求平均
	 {
		MPU_Read();

		sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
		sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
		sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);



		sensor.gyro.temp.x+= sensor.gyro.origin.x;
		sensor.gyro.temp.y+= sensor.gyro.origin.y;
		sensor.gyro.temp.z+= sensor.gyro.origin.z;
		
   }
	 cnt=200;

	 sensor.gyro.quiet.x=(sensor.gyro.temp.x/cnt);
	 sensor.gyro.quiet.y=sensor.gyro.temp.y/cnt;
	 sensor.gyro.quiet.z=sensor.gyro.temp.z/cnt;
	 
//		AT24cxx_save_Acc_Gyro_offest();
}

