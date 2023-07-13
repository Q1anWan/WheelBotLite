/*********************************************************************************
  *FileName:	SPI_LL.cpp
  *Author:  	qianwan
  *Details: 	STM32 LL-API SPI控制
  
				使用DMA时必须使用带DMA参数的初始化函数！！
			**	SPI_USE_TX_DMA == 1时开启发送DMA
				DMA发送需要在发送DMA的DMAx_IRQHandler函数中调用IRQ_Tx
				//////////////////////////////////////////////////////////////////
			**	SPI_USE_RX_DMA == 1时开启接收DMA
				DMA接收需要在接收DMA的DMAx_IRQHandler函数中调用IRQ_Rx

  *Version:  	1.3
  *Date:  		2023/05/04
  *Other:		收发加保护锁, 修改IRQ函数逻辑, 图形化实现头文件配置
  
  *Version:  	1.2.3
  *Date:  		2023/03/02
  *Other:		增加DMA接收功能, 函数实现方式重写, 代码结构重写
 
  *Version:  	1.1
  *Date:  		2022/09/03
  *Other:		增加使用宏定义开启DMA收发
**********************************************************************************/
#include "SPI_LL.h"
//#include "spi.h"

uint8_t cSPI::SPI_ExchangeOneByte(uint8_t Data)
{

	volatile uint16_t wait_cnt = 0;
	#if defined(SPI_TXDR_TXDR)
	
	LL_SPI_SetTransferSize(this->SPI,1);
	LL_SPI_Enable(this->SPI);
	LL_SPI_StartMasterTransfer(SPI2);
	
	while(!LL_SPI_IsActiveFlag_TXP(this->SPI))
	#else
	while(!LL_SPI_IsActiveFlag_TXE(this->SPI))
	#endif
	{
		if (wait_cnt++ > SPI_TIME_OVER_TIME)
		{break;}
	}
	LL_SPI_TransmitData8(this->SPI,Data);
	wait_cnt = 0;
	#if defined(SPI_TXDR_TXDR)
	while(!LL_SPI_IsActiveFlag_RXP(this->SPI))
	#else
	while(!LL_SPI_IsActiveFlag_RXNE(this->SPI))
	#endif
	{
		if (wait_cnt++ > SPI_TIME_OVER_TIME)
		{break;}
	}
	uint8_t tmp =  LL_SPI_ReceiveData8(this->SPI);
	LL_SPI_Disable(this->SPI);
	return tmp;
//	uint8_t spidb;
//	HAL_SPI_TransmitReceive(&hspi2,&Data,&spidb,1,0xFFFF);
//	return (spidb);
}

#if (SPI_USE_TX_DMA==1)
uint8_t cSPI::Transmit_DMA(uint8_t *data,uint16_t num)
{
	/*检查锁*/
	if(this->TX_Statue!=STATUE_RDY){return 1;}
	/*上锁*/
	this->TX_Statue = STATUE_BSY;

	/*配置本次传输*/
	this->DMAt.ClearFlag_TC();
	LL_DMA_SetMemoryAddress(this->DMAt.DMAx,this->DMAt.CH,(uint32_t)data);
	LL_DMA_SetDataLength(this->DMAt.DMAx,this->DMAt.CH,num);

	/*使能传输*/
	this->DMAt.EnableCH();
	return 0;
}
uint8_t cSPI::IRQ_Tx(void)
{
	this->DMAt.DisableCH();
	this->DMAt.ClearFlag_TC();
	this->TX_Statue = STATUE_RDY;
	return 0;
}
#endif
#if (SPI_USE_RX_DMA==1)
uint8_t cSPI::Receive_DMA(uint8_t *data,uint16_t num)
{
	/*检查锁*/
	if(this->RX_Statue!=STATUE_RDY){return 1;}
	/*上锁*/
	this->RX_Statue = STATUE_BSY;

	/*配置本次传输*/
	LL_SPI_ClearFlag_OVR(this->SPI);	
	this->DMAr.ClearFlag_TC();
	LL_DMA_SetMemoryAddress(this->DMAr.DMAx,this->DMAr.CH,(uint32_t)data);
	LL_DMA_SetDataLength(this->DMAr.DMAx,this->DMAr.CH,num);
	/*使能传输*/
	this->DMAr.EnableCH();
	return 0;
}

uint8_t cSPI::IRQ_Rx(void)
{
	this->DMAr.DisableCH();
	this->DMAr.ClearFlag_TC();
	this->RX_Statue = STATUE_RDY;
	return 0;
}
#endif