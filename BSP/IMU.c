#include "IMU.h"
#include "math.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "led.h"


struct _angle angle;





#define Kp 100.0f                        // 比例增益支配收敛率accelerometer/magnetometer  
#define Ki 0.003f                     // 积分增益支配执政速率陀螺仪的衔接gyroscopeases  //KP,KI需要调的
#define halfT 0.0015f                 // 采样周期的一半  本程序 4MS 采集一次  所以 halfT是2MS


//#define KpDef 0.8f
//#define KiDef 0.0005f
//#define SampleRateHalf 0.001f  

//#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.03f   // integral gain governs rate of convergence of gyroscope biases

/**************************************
 * 函数名：Get_Attitude
 * 描述  ：得到当前姿态
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void Get_Attitude(void)
{
	Prepare_Data();
	
	IMUupdate(sensor.gyro.averag.x,
						sensor.gyro.averag.y,
						sensor.gyro.averag.z,
						sensor.acc.averag.x,
						sensor.acc.averag.y,
						sensor.acc.averag.z
						,Magn_x,Magn_y,Magn_z
						);	

}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    //四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;    // 按比例缩小积分误差

//void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az)
//{
//  float norm;
////	int16_t Xr,Yr;
//  float vx, vy, vz;// wx, wy, wz;
//  float ex, ey, ez;

//  // 先把这些用得到的值算好
//  float q0q0 = q0*q0;
//  float q0q1 = q0*q1;
//  float q0q2 = q0*q2;
//  //  float q0q3 = q0*q3;//
//  float q1q1 = q1*q1;
//  //  float q1q2 = q1*q2;//
//  float q1q3 = q1*q3;
//  float q2q2 = q2*q2;
//  float q2q3 = q2*q3;
//  float q3q3 = q3*q3;
//	
//	if(ax*ay*az==0)
// 		return;
//		
//  norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
//  ax = ax *norm;
//  ay = ay * norm;
//  az = az * norm;

//  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
//  vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
//  vy = 2*(q0q1 + q2q3);
//  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

//  // error is sum of cross product between reference direction of fields and direction measured by sensors
//  ex = (ay*vz - az*vy) ;                           					 //向量外积在相减得到差分就是误差
//  ey = (az*vx - ax*vz) ;
//  ez = (ax*vy - ay*vx) ;

//  exInt = exInt + VariableParameter(ex) * ex * Ki;								  //对误差进行积分
//  eyInt = eyInt + VariableParameter(ey) * ey * Ki;
//  ezInt = ezInt + VariableParameter(ez) * ez * Ki;
//// adjusted gyroscope measurements

//  gx = gx + Kp *  VariableParameter(ex) * ex + exInt;	
//	gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
//	gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
//  								
//  // integrate quaternion rate and normalise						   //四元素的微分方程
//  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

//  // normalise quaternion
//  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
//  q0 = q0 * norm;
//  q1 = q1 * norm;
//  q2 = q2 * norm;
//  q3 = q3 * norm;

//	angle.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* RtA;
////	angle.yaw=0;
//  angle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) *RtA; // roll
//	angle.pitch = asin(-2*q1*q3 + 2*q0*q2) *RtA; // pitch


//}


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    volatile float norm;
  volatile float hx, hy, hz, bx, bz;
  volatile float vx, vy, vz, wx, wy, wz;
  volatile float ex, ey, ez;
  float temp0,temp1,temp2,temp3;
//  float temp;
//	float Xr,Yr;
  // ?????????
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
//  now = micros();  //读取时间
//  if(now < lastUpdate){ //定时器溢出过了。
//		halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);	
//		lastUpdate = now;
//		//return ;
//  }
//  else{
//		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//  	}
//  halftime = halfT;
//  lastUpdate = now;	//更新时间

//  temp = sqrt(ax*ax + ay*ay + az*az);
//  temp = (temp / 16384.0f) * 9.8f;   //转成M/S^2为单位的 
//  acc_vector = acc_vector +   //低通滤波。截止频率20hz
//  			(halfT*2.0f / (7.9577e-3f + halfT*2.0f)) * (temp - acc_vector);

  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // compute reference direction of flux
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  ezInt = ezInt + ez * Ki * halfT;

  // adjusted gyroscope measurements
  gx = gx + (Kp*ex + exInt);
  gy = gy + (Kp*ey + eyInt);
  gz = gz + (Kp*ez + ezInt);

  }

  // integrate quaternion rate and normalise
  temp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  temp1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  temp2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  temp3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // normalise quaternion
  norm = invSqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
  q0 = temp0 * norm;
  q1 = temp1 * norm;
  q2 = temp2 * norm;
  q3 = temp3 * norm;
	

  angle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) ; // roll
	angle.pitch = asin(-2*q1*q3 + 2*q0*q2) ; // pitch
	
	angle.yaw= atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1);
	
//	angle.roll= LPF_1st(old_roll ,angle.roll ,0.8f);
//	old_roll= angle.roll;
//	 
//	angle.pitch= LPF_1st(old_pitch ,angle.pitch ,0.8f);
//	old_pitch= angle.pitch;
	
	
//		if(tempyaw>0)
//		angle.yaw=tempyaw;
//	else
//		angle.yaw=360+tempyaw;
//	//          关于地磁如何进行倾角补偿                       //    
//	//参考  http://baike.baidu.com/view/1239157.htm?fr=aladdin    //

//	Xr = mx * COS(angle.pitch*RtA) + my * SIN(-angle.pitch*RtA) * SIN(-angle.roll*RtA) - mz * COS(angle.roll*RtA) * SIN(-angle.pitch*RtA);
//	Yr = my * COS(angle.roll*RtA) + mz * SIN(-angle.roll*RtA);
//	
//	angle.yaw = atan2((double)Yr,(double)Xr) ; // yaw 
	angle.roll *= RtA;
	angle.pitch *= RtA;
	angle.yaw*= RtA;

	LEDXToggle(1);
}











