#include "i2c.h"
#include "MPU6050.h"
#include "IMU.h"
#include "HMC5883L.h"
	

u8		 mpu6050_buffer[15];

struct _sensor sensor;


////////////////////////////////////////////////////////////////////////////////// 	


//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{ 
	u8 res;


	IIC_ADD_write(MPU_ADDR,MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	
	IIC_ADD_write(MPU_ADDR,0x19, 0x07);    //�����ǲ�����
//	IIC_ADD_write(MPU_ADDR,MPU_CFG_REG, 0x04);        //25Hz 
	
	IIC_ADD_write(MPU_ADDR,MPU_INTBP_CFG_REG, 0x42);   //ʹ����·I2C
	IIC_ADD_write(MPU_ADDR,MPU_USER_CTRL_REG, 0x40);     //ʹ����·I2C
	IIC_ADD_write(MPU_ADDR,MPU_CFG_REG, 0x02);     //��ͨ�˲�
	IIC_ADD_write(MPU_ADDR,MPU_GYRO_CFG_REG, 0x10);      //+-1000
	IIC_ADD_write(MPU_ADDR,MPU_ACCEL_CFG_REG, 0x09);
	
	
	res=IIC_ADD_read(MPU_ADDR,MPU_DEVICE_ID_REG);
	if(res==0x68)//����ID��ȷ
	{
		return 0;
 	}else 
	{
		return 1;
	}
}


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
	

	
	sensor.gyro.radian.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x )  ;
	sensor.gyro.radian.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y ) ;
	sensor.gyro.radian.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z ) ;

	if(sensor.acc.CALIBRATE==1)
	{
		MPU_Acc_Offset();

		sensor.acc.CALIBRATE=0;
	}

	if(sensor.gyro.CALIBRATE==1)
	{
		MPU_Gyro_Offset();	

		sensor.gyro.CALIBRATE=0;
	}
}



/*	
	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
			
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬
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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
				
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬
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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
				
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬
	return x_now;                
 }



/*====================================================================================================*
**���� : LPF_1st
**���� : һ�׵�ͨ�˲�
**���� :  
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}




void Prepare_Data(void)
{
	static float x,y,z;
	
	MPU_Updata();
    Multiple_Read_HMC5883L();
	sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,KALMAN_Q,KALMAN_R);  // ACC X�Ῠ�����˲�
	sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,KALMAN_Q,KALMAN_R);  // ACC Y�Ῠ�����˲�
	sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,KALMAN_Q,KALMAN_R);  // ACC Z�Ῠ�����˲�

// ������һ�׵�ͨ�˲�
 	sensor.gyro.radian1.x = LPF_1st(x,sensor.gyro.radian.x ,0.386f);	x = sensor.gyro.radian1.x;    //��0.2 - 0.4��
 	sensor.gyro.radian1.y = LPF_1st(y,sensor.gyro.radian.y ,0.386f);	y = sensor.gyro.radian1.y;
	sensor.gyro.radian1.z = LPF_1st(z,sensor.gyro.radian.z  ,0.386f);	z = sensor.gyro.radian1.z;//
	sensor.gyro.averag.x = sensor.gyro.radian1.x *Gyro_Gr;
	sensor.gyro.averag.y = sensor.gyro.radian1.y *Gyro_Gr;
	sensor.gyro.averag.z = sensor.gyro.radian1.z *Gyro_Gr;

}	



/*
 * ��������MPU_Acc_Offset
 * ����  ��MPU����У׼
 * ����  ��У׼����
 * ���  ����
 */ 
void MPU_Acc_Offset(void)
{
	u16 cnt=200;
	sensor.acc.temp.x=0;
	sensor.acc.temp.y=0;
	sensor.acc.temp.z=0;

	sensor.gyro.averag.x=0;    //���ƫ������
	sensor.gyro.averag.y=0;  
	sensor.gyro.averag.z=0;
	while(cnt--)       //ѭ���ɼ�2000��   ��ƽ��
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

	sensor.acc.averag.x=0;    //���ƫ������
	sensor.acc.averag.y=0;  
	sensor.acc.averag.z=0;

	while(cnt--)       //ѭ���ɼ�2000��   ��ƽ��
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

