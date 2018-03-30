#include "IMU.h"
#include "math.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "led.h"


struct _angle angle;

#define Kp 100.0f                        // 比例增益支配收敛率accelerometer/magnetometer  
#define Ki 0.003f                     // 积分增益支配执政速率陀螺仪的衔接gyroscopeases  //KP,KI需要调的
#define halfT 0.0015f                 // 采样周期的一半  本程序 3MS 采集一次  所以 halfT是1.5MS

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
						sensor.acc.averag.z,
						Mag.n_x,
						Mag.n_y,
						Mag.n_z
						);	

}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    //四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;    // 按比例缩小积分误差


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	volatile float norm;
	volatile float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez;
	float temp0,temp1,temp2,temp3;

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



	norm = sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	norm = sqrt(mx*mx + my*my + mz*mz);          
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
	norm = sqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
	q0 = temp0 * norm;
	q1 = temp1 * norm;
	q2 = temp2 * norm;
	q3 = temp3 * norm;


	angle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) ; // roll
	angle.pitch = asin(-2*q1*q3 + 2*q0*q2) ; // pitch
	angle.yaw= atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1);//yaw

	angle.roll *= RtA;
	angle.pitch *= RtA;
	angle.yaw*= RtA;

	LEDXToggle(1);
}











