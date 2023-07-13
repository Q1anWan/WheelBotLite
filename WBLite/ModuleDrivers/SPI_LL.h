/*********************************************************************************
  *FileName:	SPI_LL.h
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
#ifndef SPI_LL_H
#define SPI_LL_H
#include "main.h"
#include "spi.h"
#ifdef __cplusplus

// <<< Use Configuration Wizard in Context Menu >>>

// <o>��ʱ�ȴ�����
//  <i>Ĭ��ֵ: 5000
#define SPI_TIME_OVER_TIME 0xFFff

// <c1>���ô���DMA����
//  <i>���ô���DMA����
//#define SPI_USE_TX_DMA 1
// </c>

// <c1>���ô���DMA����
// <i> ���ô���DMA����
//#define SPI_USE_RX_DMA 1
// </c>

// <<< end of configuration section >>>


#if (SPI_USE_TX_DMA)||(SPI_USE_RX_DMA)
#include "DMAFlag_LL.h"
#endif
/*! 
 *  @brief      ���SPI����
 *  @brief		Ϊ�����ṩSPI��ʼ�������ݽ���������CS���ơ�MISO���ƵȻ�������
 */
class cSPI
{
	protected:
	SPI_TypeDef 	*SPI;
	GPIO_TypeDef 	*CS_Port;
	uint32_t 		 CS_Pin;

	/*״̬��, ������, ȷ���̰߳�ȫ*/
	enum eStatue
	{
		STATUE_RDY = 0,
		STATUE_BSY = 1
	};
	eStatue RX_Statue = STATUE_RDY;
	eStatue TX_Statue = STATUE_RDY;
	
	public:
	uint8_t SPI_ExchangeOneByte(uint8_t Data);
	inline void CS_0(void)
	{LL_GPIO_ResetOutputPin(CS_Port,CS_Pin);}
	inline void CS_1(void)
	{LL_GPIO_SetOutputPin(CS_Port,CS_Pin);}

	public:
	/*SPI_Init��������ݲ�ͬ�Ĵ������أ����������ѡ��DMAģʽ*/
	/*��ʹ��DMA��������ʽ*/
	void SPI_Init(SPI_TypeDef *SPI, GPIO_TypeDef *CS_Port, uint32_t CS_Pin)
	{
		this->SPI = SPI;
		this->CS_Port = CS_Port;
		this->CS_Pin = CS_Pin;
		this->CS_1();
//		LL_SPI_Enable(this->SPI);
	}

	/*ʹ��DMA����*/	
	#if (SPI_USE_TX_DMA==1)
	protected:
	cDMA DMAt;
	public:
	/*ʹ��DMA���͵�������ʽ*/
	void SPI_Init(SPI_TypeDef *SPI, GPIO_TypeDef *CS_Port, uint32_t CS_Pin,
				  DMA_TypeDef *DMAt, uint32_t DMA_CHt)
	{
		this->SPI = SPI;
		this->CS_Port = CS_Port;
		this->CS_Pin = CS_Pin;
		this->CS_1();	

		this->DMAt.Config(DMAt,DMA_CHt);
		this->DMAt.ClearFlag_HT();
		this->DMAt.ClearFlag_TE();
		
		LL_SPI_EnableDMAReq_TX(this->SPI);
		#if defined(SPI_TXDR_TXDR)/*H7ϵ��*/
		LL_DMA_SetPeriphAddress(this->DMAt.DMAx,this->DMAt.CH,(uint32_t)&this->SPI->TXDR);
		#else
		LL_DMA_SetPeriphAddress(this->DMAt.DMAx,this->DMAt.CH,(uint32_t)&this->SPI->DR);
		#endif
		LL_DMA_EnableIT_TC(this->DMAt.DMAx,this->DMAt.CH);
		LL_SPI_Enable(SPI);
	}
	uint8_t Transmit_DMA(uint8_t *data,uint16_t num);
	uint8_t IRQ_Tx(void);
	#endif
	
	/*ʹ��DMA����*/
	#if (SPI_USE_RX_DMA==1)
	protected:
	cDMA DMAr;
	public:
	/*ʹ��DMA���յ�������ʽ,AnyValueû�����壬�����ڱ�����ʶ������*/
	void SPI_Init(SPI_TypeDef *SPI, GPIO_TypeDef *CS_Port, uint32_t CS_Pin,
				  DMA_TypeDef *DMAr, uint32_t DMA_CHr, uint8_t AnyValue)
	{
		this->SPI = SPI;
		this->CS_Port = CS_Port;
		this->CS_Pin = CS_Pin;
		this->CS_1();
		
		this->DMAr.Config(DMAr,DMA_CHr);
		this->DMAr.ClearFlag_HT();
		this->DMAr.ClearFlag_TE();
		
		LL_SPI_EnableDMAReq_RX(this->SPI);
		#if defined(SPI_TXDR_TXDR)/*H7ϵ��*/
		LL_DMA_SetPeriphAddress(this->DMAr.DMAx,this->DMAr.CH,(uint32_t)&this->SPI->TXDR);
		#else
		LL_DMA_SetPeriphAddress(this->DMAr.DMAx,this->DMAr.CH,(uint32_t)&this->SPI->DR);
		#endif
		LL_DMA_EnableIT_TC(this->DMAr.DMAx,this->DMAr.CH);
		LL_SPI_Enable(SPI);
	}
	uint8_t Receive_DMA(uint8_t *data,uint16_t num);
	uint8_t IRQ_Rx(void);
	#endif
	
	/*ʹ��DMA�շ�*/
	#if (SPI_USE_TX_DMA==1)&&(SPI_USE_RX_DMA==1)
	public:
	/*ʹ��DMA�շ���������ʽ*/
	void SPI_Init(SPI_TypeDef *SPI, GPIO_TypeDef *CS_Port, uint32_t CS_Pin, 
				  DMA_TypeDef *DMAr, uint32_t DMA_CHr, 
				  DMA_TypeDef *DMAt, uint32_t DMA_CHt)
	{
		this->SPI = SPI;
		this->CS_Port = CS_Port;
		this->CS_Pin = CS_Pin;
		this->CS_1();
		
		this->DMAr.Config(DMAr,DMA_CHr);
		this->DMAt.Config(DMAt,DMA_CHt);
		this->DMAr.ClearFlag_HT();
		this->DMAr.ClearFlag_TE();
		this->DMAt.ClearFlag_HT();
		this->DMAt.ClearFlag_TE();
		
		LL_SPI_EnableDMAReq_RX(this->SPI);
		LL_SPI_EnableDMAReq_TX(this->SPI);
		
		#if defined(SPI_TXDR_TXDR)/*H7ϵ��*/
		LL_DMA_SetPeriphAddress(this->DMAr.DMAx,this->DMAr.CH,(uint32_t)&this->SPI->RXDR);
		LL_DMA_SetPeriphAddress(this->DMAt.DMAx,this->DMAt.CH,(uint32_t)&this->SPI->TXDR);
		#else
		LL_DMA_SetPeriphAddress(this->DMAr.DMAx,this->DMAr.CH,(uint32_t)&this->SPI->DR);
		LL_DMA_SetPeriphAddress(this->DMAt.DMAx,this->DMAt.CH,(uint32_t)&this->SPI->DR);
		#endif

		LL_DMA_EnableIT_TC(this->DMAr.DMAx,this->DMAr.CH);
		LL_DMA_EnableIT_TC(this->DMAt.DMAx,this->DMAt.CH);
		LL_SPI_Enable(this->SPI);
	}
	#endif
};


#endif
#endif
