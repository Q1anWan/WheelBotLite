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
#ifndef ICM42688_H
#define ICM42688_H
#include <main.h>
#ifdef LL_SPI_MODE_MASTER
#include "SPI_LL.h"
#else
#include "spi.h"
#endif

#ifdef __cplusplus

/*G = 9.8011 in Dalian*/
#define LSB_ACC_16G		0.0047856934f
#define LSB_ACC_8G		0.0023928467f
#define LSB_ACC_4G		0.0011964233f
#define LSB_ACC_2G		0.00059821167f

/*Turn Into Radian*/
#define LSB_GYRO_2000_R	0.0010652644f
#define LSB_GYRO_1000_R	0.00053263222f
#define LSB_GYRO_500_R	0.00026631611f
#define LSB_GYRO_250_R	0.00013315805f
#define LSB_GYRO_125D_R	0.000066579027f

#ifndef LL_SPI_MODE_MASTER
class cSPI
{
	protected:
	SPI_HandleTypeDef *SPI;
	GPIO_TypeDef 	*CS_Port;
	uint32_t 		 CS_Pin;

	public:
	uint8_t SPI_ExchangeOneByte(uint8_t Data)
	{
		uint8_t buf;
		HAL_SPI_TransmitReceive(this->SPI,&Data,&buf,1,5);
		return buf;
	}
	
	inline void SPI_ExchangeOneByte(uint8_t *TxBuf, uint8_t *RxBuf, uint16_t len, uint16_t OverTime)
	{HAL_SPI_TransmitReceive(this->SPI, TxBuf, RxBuf, len, OverTime);}

	inline void CS_0(void)
	{LL_GPIO_ResetOutputPin(CS_Port,CS_Pin);}
	inline void CS_1(void)
	{LL_GPIO_SetOutputPin(CS_Port,CS_Pin);}
	void SPI_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint32_t CS_Pin)
	{
		this->SPI = hspi;
		this->CS_Port = CS_Port;
		this->CS_Pin = CS_Pin;
		this->CS_1();
	}
};
#endif

class cICM42688 : public cSPI
{
	public:
		
	int16_t Accel[3]={0};//XYZ
	int16_t Gyro[3]={0};//XYZ
	float Temperature=0;
	float LSB_ACC_GYRO[2]={0};
	
	#ifndef LL_SPI_MODE_MASTER
	uint8_t rubuf[20]={0}; //Tolit
	uint8_t ReadReg(uint8_t Reg)
	{
		uint8_t Send[2] = {Reg, 0xFF};
		uint8_t Read[2] = {0};
		
		this->CS_0();
		HAL_SPI_TransmitReceive(this->SPI, Send, Read, 2, 2);
		this->CS_1();
		return Read[1];
	}
	
	void ReadReg(uint8_t Reg, uint8_t* Data, uint8_t num)
	{
		Reg |= 0x80;

		this->CS_0();
		this->SPI_ExchangeOneByte(Reg);
		this->SPI_ExchangeOneByte(this->rubuf, Data, num, 2);
		this->CS_1();
	}
	
	void WriteReg( uint8_t Reg, uint8_t  Data)
	{
		this->rubuf[18]=Reg;
		this->rubuf[19]=Data;
		
		this->CS_0();
		HAL_SPI_TransmitReceive(this->SPI, &this->rubuf[18], this->rubuf, 2, 2);
		this->CS_1();
	}

	void WriteReg( uint8_t Reg, uint8_t  *Data, uint8_t num)
	{
		this->CS_0();
		this->SPI_ExchangeOneByte(Reg);
		this->SPI_ExchangeOneByte(Data, this->rubuf, num, 2);
		this->CS_1();
	}
	#else
	uint8_t ReadReg(uint8_t Reg)
	{
		uint8_t Read = 0xFF;
		Reg |= 0x80;
		this->CS_0();
		this->SPI_ExchangeOneByte(Reg);
		Read = this->SPI_ExchangeOneByte(0xFF);
		this->CS_1();
		return Read;
	}

	void ReadReg(uint8_t Reg, uint8_t* Data, uint8_t num)
	{
		Reg |= 0x80;

		this->CS_0();
		this->SPI_ExchangeOneByte(Reg);
		for(uint8_t i = 0; i < num; i++)
		{Data[i] = this->SPI_ExchangeOneByte(0x00);}
		this->CS_1();
	}
	
	void cICM42688::WriteReg( uint8_t Reg, uint8_t  Data)
	{
		this->CS_0();
		this->SPI_ExchangeOneByte(Reg);
		this->SPI_ExchangeOneByte(Data);
		this->CS_1();
	}

	void cICM42688::WriteReg( uint8_t Reg, uint8_t  *Data, uint8_t num)
	{
		this->CS_0();
		this->SPI_ExchangeOneByte(Reg);
		for(uint8_t i = 0; i < num; i++)
		{this->SPI_ExchangeOneByte(Data[i]);}
		this->CS_1();
	}
	#endif
	
	void	ReadAccel(void);
	void	ReadGyro(void);
	void	ReadAccelGyro(void);
	void 	ReadTem(void);
	
};


extern "C" {

}

#endif
#endif