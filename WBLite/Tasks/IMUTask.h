#ifndef IMU_TASK
#define IMU_TASK
#include "main.h"
#include "ICM42688.h"
#include "PID.h"

class cIMU : public cICM42688,public cPIDInc
{	
	public:
	cIMU()
	{
		PID_Init();
	
	}

	float GyroCalVal[3]	= {0};
	float AccelCalVal[3]	= {0};
	
	void SetLSB(float Accel, float Gyro)
	{
		this->cICM42688::LSB_ACC_GYRO[0] = Accel;
		this->cICM42688::LSB_ACC_GYRO[1] = Gyro;
	}
	
	void PID_Init(void)
	{
		this->Ref = 41.0f;
		this->Kp = 200.0f;
		this->Ki = 2.0f;
		this->Kd = 5.0f;
		this->IN_RANGE_EN_D = 3.0f;	//开启微分项范围 值为0时始终开启
		this->IN_RANGE_EN_I = 0.0f;	//开启积分项范围 值为0时始终开启
		this->MaxOutValue=500.0f;		//输出限幅
		this->MinOutValue=0.0f;
	}
	
};

class cINS
{
	public:
		
	cIMU *IMU;
	
	float Accel[3];
	float Gyro[3];
	float *Temperature;
	
	float Q[4]={1.0f,0,0,0};
	
	float ABS_Chasis = 0.0f;
	
	inline void SetIMU(cIMU *pIMU)
	{this->IMU = pIMU;}
};



extern "C"{
	void EXTI0_IRQHandler(void);
	void EXTI1_IRQHandler(void);
}
#endif