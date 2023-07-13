/*********************************************************************************
  *FileName:		ICM42688.h
  *Author:  		qianwan
  *Detail: 			ICM42688-P驱动

  *Version:  		1.1
  *Date:  			2023/07/13
  *Describe:		支持SPI HAL
  
  *Version:  		1.0
  *Date:  			2023/03/10
  *Describe:		新建项目
**********************************************************************************/
#include "ICM42688.h"

void cICM42688::ReadAccel(void)
{
	uint8_t AccelBuf[6];
	this->ReadReg(0x1F,AccelBuf,6);
	this->Accel[0] = (AccelBuf[0]<<8 | AccelBuf[1]);
	this->Accel[1] = (AccelBuf[2]<<8 | AccelBuf[3]);
	this->Accel[2] = (AccelBuf[4]<<8 | AccelBuf[5]);
}

void cICM42688::ReadGyro(void)
{
	uint8_t GyroBuf[6];
	this->ReadReg(0x25,GyroBuf,6);
	this->Gyro[0] = (GyroBuf[0]<<8 | GyroBuf[1]);
	this->Gyro[1] = (GyroBuf[2]<<8 | GyroBuf[3]);
	this->Gyro[2] = (GyroBuf[4]<<8 | GyroBuf[5]);
};

void cICM42688::ReadAccelGyro(void)
{
	uint8_t Buf[12];
	this->ReadReg(0x1F,Buf,12);
	this->Accel[0] = (int16_t)(Buf[0]<<8 | Buf[1]);
	this->Accel[1] = (int16_t)(Buf[2]<<8 | Buf[3]);
	this->Accel[2] = (int16_t)(Buf[4]<<8 | Buf[5]);
	this->Gyro[0]  = (int16_t)(Buf[6]<<8 | Buf[7]);
	this->Gyro[1]  = (int16_t)(Buf[8]<<8 | Buf[9]);
	this->Gyro[2]  = (int16_t)(Buf[10]<<8| Buf[11]);
}

void cICM42688::ReadTem(void)
{
	uint8_t buf[2]={0};
	int16_t raw_tmp;

	this->ReadReg(0x1D,buf,2);
	
	raw_tmp = (int16_t)((buf[0]<<8)|(buf[1]));
	//低通滤波器截止频率:100Hz
	//this->Temperature = 0.1*(((float)raw_tmp/132.48f)+25.0f) + 0.9*this->Temperature;
	this->Temperature = ((float)raw_tmp/1324.8f)+ 2.5f + 0.9*this->Temperature;
}

