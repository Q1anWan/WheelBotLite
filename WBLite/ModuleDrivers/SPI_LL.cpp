/*********************************************************************************
  *FileName:	SPI_LL.cpp
  *Author:  	qianwan
  *Details: 	STM32 LL-API SPI����
  
				ʹ��DMAʱ����ʹ�ô�DMA�����ĳ�ʼ����������
			**	SPI_USE_TX_DMA == 1ʱ��������DMA
				DMA������Ҫ�ڷ���DMA��DMAx_IRQHandler�����е���IRQ_Tx
				//////////////////////////////////////////////////////////////////
			**	SPI_USE_RX_DMA == 1ʱ��������DMA
				DMA������Ҫ�ڽ���DMA��DMAx_IRQHandler�����е���IRQ_Rx

  *Version:  	1.3
  *Date:  		2023/05/04
  *Other:		�շ��ӱ�����, �޸�IRQ�����߼�, ͼ�λ�ʵ��ͷ�ļ�����
  
  *Version:  	1.2.3
  *Date:  		2023/03/02
  *Other:		����DMA���չ���, ����ʵ�ַ�ʽ��д, ����ṹ��д
 
  *Version:  	1.1
  *Date:  		2022/09/03
  *Other:		����ʹ�ú궨�忪��DMA�շ�
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
	/*�����*/
	if(this->TX_Statue!=STATUE_RDY){return 1;}
	/*����*/
	this->TX_Statue = STATUE_BSY;

	/*���ñ��δ���*/
	this->DMAt.ClearFlag_TC();
	LL_DMA_SetMemoryAddress(this->DMAt.DMAx,this->DMAt.CH,(uint32_t)data);
	LL_DMA_SetDataLength(this->DMAt.DMAx,this->DMAt.CH,num);

	/*ʹ�ܴ���*/
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
	/*�����*/
	if(this->RX_Statue!=STATUE_RDY){return 1;}
	/*����*/
	this->RX_Statue = STATUE_BSY;

	/*���ñ��δ���*/
	LL_SPI_ClearFlag_OVR(this->SPI);	
	this->DMAr.ClearFlag_TC();
	LL_DMA_SetMemoryAddress(this->DMAr.DMAx,this->DMAr.CH,(uint32_t)data);
	LL_DMA_SetDataLength(this->DMAr.DMAx,this->DMAr.CH,num);
	/*ʹ�ܴ���*/
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