#ifndef _HMC5883L_H_
#define _HMC5883L_H_
#include "stm32f0xx.h"


/*宏定义------------------------------------------------------------------*/
#define	MPU6050_Addr   0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define INT_PIN_CFG     0x37    //设置旁路有效 打开值：0x42 AUX_DA的辅助I2C
#define USER_CTRL       0x6A    //用户配置寄存器 打开值：0x40  AUX_DA的辅助I2C
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)




#define ABS(x) ((x)>=0?(x):(-(x)))

#define	HMC5883L_Addr   								 0x3C	//磁场传感器器件地址   
#define HMC5883L_ConfigurationRegisterA  0x00
#define HMC5883L_ConfigurationRegisterB  0x01
#define HMC5883L_ModeRegister            0x02
#define HMC5883L_Output_X_MSB            0x03
#define HMC5883L_Output_X_LSB 					 0x04
#define HMC5883L_Output_Z_MSB            0x05
#define HMC5883L_Output_Z_LSB 					 0x06
#define HMC5883L_Output_Y_MSB            0x07
#define HMC5883L_Output_Y_LSB 					 0x08
#define HMC5883L_StatusRegister					 0x09
#define HMC5883L_ID_A										 0x0A
#define HMC5883L_ID_B 									 0x0B
#define HMC5883L_ID_C 									 0x0C

#define HMC5883L_OFFSET_X   (9)
#define HMC5883L_OFFSET_Y   (149)

#define CalThreshold 0 


extern int Magn_x,Magn_y,Magn_z;
extern u8 HMC5883_cnt;

u8 Init_HMC5883L(void);
void Multiple_Read_HMC5883L(void);
void HMC5883L_Start_Calib(void);
void HMC5883L_Save_Calib(void);



#endif


