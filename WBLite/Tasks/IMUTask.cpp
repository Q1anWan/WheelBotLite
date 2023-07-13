#include "IMUTask.h"
#include "MahonyAHRS.h"
#include "Data_Exchange.h"
#include "QCSLite.h"

static void IMU_PreInit(void);
static void IMU_Init(void);

extern void ReadIMUCal(float *GyroC, float *AccelC);
extern void WriteIMUCal(float *GyroC, float *AccelC);
static void GYROCal(void);

TX_THREAD INSThread;
TX_SEMAPHORE IMUSem;
uint8_t INSThreadStack[2048]={0};
cINS *INS = 0;


uint8_t WhoAmI = 0;
void INSThreadFun(ULONG initial_input)
{
	cINS INS_t;
	INS = &INS_t;
	
	
	cIMU IMU_t;

	IMU_t.SPI_Init(&hspi2, GPIOB, LL_GPIO_PIN_9);
	INS->SetIMU(&IMU_t);
	INS->Temperature = &IMU_t.Temperature;
	IMU_PreInit();
	
	/*读取零漂数据*/
	ReadIMUCal(INS->IMU->GyroCalVal,INS->IMU->AccelCalVal);
	

	ULONG Tim = 0;
	while(INS->IMU->Temperature<40.5f)
	{
		tx_thread_sleep(20);
		if(tx_time_get()-Tim>5000)
		{break;}
	}

	IMU_Init();
	tx_thread_sleep(1000);
	
	float ZAxis[3] = {0,0,1.0f};
	float YAxis[3] = {0.0f,1.0f,0};
	float XAxis[3] = {1.0f,0.0f,0};
	uint16_t cal=0;
	for(;;)
	{
		if(tx_semaphore_get(&IMUSem,10)==0x0D)
		{
			/*42688掉线*/
			tx_semaphore_get(&IMUSem,TX_WAIT_FOREVER);
		}
		//GYROCal();//calibiration
		/*互补滤波迭代四元数*/
		MahonyAHRSupdateINS(INS->Q,-INS->Gyro[1],INS->Gyro[0],INS->Gyro[2],-INS->Accel[1],INS->Accel[0],INS->Accel[2]);
	}
}


TX_THREAD TemThread;
uint8_t TemThreadStack[512]={0};
void TemThreadFun (ULONG initial_input)
{
	ULONG Ticker = 0;
	uint32_t PWM=0;
	
	LL_TIM_CC_EnableChannel(TIM16,LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM16);
	LL_TIM_EnableCounter(TIM16);
	
	/*等待IMU预初始化*/
	tx_thread_sleep(50);
	
	for(;;)
	{
		Ticker = tx_time_get();		
		
		PWM = (uint32_t)INS->IMU->PID_Cal(*INS->Temperature);
		LL_TIM_OC_SetCompareCH1(TIM16,PWM);
		tx_thread_sleep_until(&Ticker,20);
	}
}

void EXTI1_IRQHandler(void)
{
	INS->IMU->ReadAccelGyro();
	INS->IMU->ReadTem();
	INS->Accel[0] = INS->IMU->Accel[0]*INS->IMU->LSB_ACC_GYRO[0] +  INS->IMU->AccelCalVal[0];
	INS->Accel[1] = INS->IMU->Accel[1]*INS->IMU->LSB_ACC_GYRO[0] +  INS->IMU->AccelCalVal[1];
	INS->Accel[2] = INS->IMU->Accel[2]*INS->IMU->LSB_ACC_GYRO[0] +  INS->IMU->AccelCalVal[2];
	INS->Gyro[0] = 	INS->IMU->Gyro[0]*INS->IMU->LSB_ACC_GYRO[1]  +  INS->IMU->GyroCalVal[0];
	INS->Gyro[1] = 	INS->IMU->Gyro[1]*INS->IMU->LSB_ACC_GYRO[1]  +  INS->IMU->GyroCalVal[1];
	INS->Gyro[2] = 	INS->IMU->Gyro[2]*INS->IMU->LSB_ACC_GYRO[1]  +  INS->IMU->GyroCalVal[2];
	tx_semaphore_put(&IMUSem);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
}
void EXTI0_IRQHandler(void)
{
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}


static void IMU_PreInit(void)
{
	uint8_t buf = 0;
	/*指定Bank0*/
	INS->IMU->WriteReg(0x76,0x00);
	/*软重启*/
	INS->IMU->WriteReg(0x11,0x01);tx_thread_sleep(5);
	/*读取中断位 切换SPI*/
	buf = INS->IMU->ReadReg(0x2D);
	/*指定Bank0*/
	INS->IMU->WriteReg(0x76,0x00);
	/*Gyro设置*/
	INS->IMU->WriteReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	INS->IMU->WriteReg(0x50,0x06);//16G 1KHz
	/*电源管理*/
	INS->IMU->WriteReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
}

//IMU初始化 用于正常工作
static void IMU_Init(void)
{
	uint8_t buf = 0;
	/*指定Bank0*/
	INS->IMU->WriteReg(0x76,0x00);
	/*软重启*/
	INS->IMU->WriteReg(0x11,0x01);tx_thread_sleep(5);
	/*读取中断位 切换SPI*/
	buf = INS->IMU->ReadReg(0x2D);

	
	/*指定Bank0*/
	INS->IMU->WriteReg(0x76,0x00);
	/*中断输出设置*/
	INS->IMU->WriteReg(0x14,0x12);//INT1 INT2 脉冲模式，低有效
	/*Gyro设置*/
	INS->IMU->WriteReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	INS->IMU->WriteReg(0x50,0x06);//16G 1KHz
	/*LSB设置*/
	INS->IMU->cICM42688::LSB_ACC_GYRO[0] = LSB_ACC_16G;
	INS->IMU->cICM42688::LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
	/*Tem设置&Gyro_Config1*/
	INS->IMU->WriteReg(0x51,0x56);//BW 82Hz Latency = 2ms
	/*GYRO_ACCEL_CONFIG0*/
	INS->IMU->WriteReg(0x52,0x11);//1BW
	/*ACCEL_CONFIG1*/
	INS->IMU->WriteReg(0x53,0x0D);//Null
	/*INT_CONFIG0*/
	INS->IMU->WriteReg(0x63,0x00);//Null
	/*INT_CONFIG1*/
	INS->IMU->WriteReg(0x64,0x00);//中断引脚正常启用
	/*INT_SOURCE0*/
	INS->IMU->WriteReg(0x65,0x08);//DRDY INT1
	/*INT_SOURCE1*/
	INS->IMU->WriteReg(0x66,0x00);//Null
	/*INT_SOURCE3*/
	INS->IMU->WriteReg(0x68,0x00);//Null
	/*INT_SOURCE3*/
	INS->IMU->WriteReg(0x69,0x00);//Null
	
/*****抗混叠滤波器@536Hz*****/
	
	/*GYRO抗混叠滤波器配置*/
	/*指定Bank1*/
	INS->IMU->WriteReg(0x76,0x01);
	/*GYRO抗混叠滤波器配置*/
	INS->IMU->WriteReg(0x0B,0xA0);//开启抗混叠和陷波滤波器
	INS->IMU->WriteReg(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
	INS->IMU->WriteReg(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
	INS->IMU->WriteReg(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
	
	/*ACCEL抗混叠滤波器配置*/
	/*指定Bank2*/
	INS->IMU->WriteReg(0x76,0x02);
	/*ACCEL抗混叠滤波器配置*/
	INS->IMU->WriteReg(0x03,0x18);//开启滤波器 ACCEL_AFF_DELT 12 (default 24)
	INS->IMU->WriteReg(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
	INS->IMU->WriteReg(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

/*****自定义滤波器1号@111Hz*****/

	/*指定Bank0*/
	INS->IMU->WriteReg(0x76,0x00);
	/*滤波器顺序*/
	INS->IMU->WriteReg(0x51,0x12);//GYRO滤波器1st
	INS->IMU->WriteReg(0x53,0x05);//ACCEL滤波器1st
	/*滤波器设置*/
	INS->IMU->WriteReg(0x52,0x33);//111Hz 03

//	NVIC_EnableIRQ(EXTI2_IRQn);
//	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
//	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
	
	NVIC_EnableIRQ(EXTI1_IRQn);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);

	
	/*指定Bank0*/
	INS->IMU->WriteReg(0x76,0x00);
	/*电源管理*/
	INS->IMU->WriteReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
}

static void GYROCal(void)
{
	static int64_t GYROCC[3];
	static uint32_t cal;
	if(cal<(uint32_t)60000L)
	{
		GYROCC[0]+=INS->IMU->cICM42688::Gyro[0];
		GYROCC[1]+=INS->IMU->cICM42688::Gyro[1];
		GYROCC[2]+=INS->IMU->cICM42688::Gyro[2];
		++cal;
	}
	else
	{
		float calval[6]={0};
		volatile uint8_t abc = 1;
		calval[0] = -(double)GYROCC[0]*INS->IMU->LSB_ACC_GYRO[1]/30000.0f;
		calval[1] = -(double)GYROCC[1]*INS->IMU->LSB_ACC_GYRO[1]/30000.0f;
		calval[2] = -(double)GYROCC[2]*INS->IMU->LSB_ACC_GYRO[1]/30000.0f;
		WriteIMUCal(calval,calval+3);
		while(abc){;}
	}
}