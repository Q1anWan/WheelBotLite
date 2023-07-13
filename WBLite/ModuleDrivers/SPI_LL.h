/*********************************************************************************
  *FileName:	SPI_LL.h
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
#ifndef SPI_LL_H
#define SPI_LL_H
#include "main.h"
#include "spi.h"
#ifdef __cplusplus

// <<< Use Configuration Wizard in Context Menu >>>

// <o>超时等待周期
//  <i>默认值: 5000
#define SPI_TIME_OVER_TIME 0xFFff

// <c1>启用串口DMA发送
//  <i>启用串口DMA发送
//#define SPI_USE_TX_DMA 1
// </c>

// <c1>启用串口DMA接收
// <i> 启用串口DMA接收
//#define SPI_USE_RX_DMA 1
// </c>

// <<< end of configuration section >>>


#if (SPI_USE_TX_DMA)||(SPI_USE_RX_DMA)
#include "DMAFlag_LL.h"
#endif
/*! 
 *  @brief      软件SPI基类
 *  @brief		为外设提供SPI初始化、数据交换函数、CS控制、MISO控制等基本方法
 */
class cSPI
{
	protected:
	SPI_TypeDef 	*SPI;
	GPIO_TypeDef 	*CS_Port;
	uint32_t 		 CS_Pin;

	/*状态量, 构成锁, 确保线程安全*/
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
	/*SPI_Init函数会根据不同的传参重载，允许针对性选择DMA模式*/
	/*不使用DMA的重载形式*/
	void SPI_Init(SPI_TypeDef *SPI, GPIO_TypeDef *CS_Port, uint32_t CS_Pin)
	{
		this->SPI = SPI;
		this->CS_Port = CS_Port;
		this->CS_Pin = CS_Pin;
		this->CS_1();
//		LL_SPI_Enable(this->SPI);
	}

	/*使用DMA发送*/	
	#if (SPI_USE_TX_DMA==1)
	protected:
	cDMA DMAt;
	public:
	/*使用DMA发送的重载形式*/
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
		#if defined(SPI_TXDR_TXDR)/*H7系列*/
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
	
	/*使用DMA接收*/
	#if (SPI_USE_RX_DMA==1)
	protected:
	cDMA DMAr;
	public:
	/*使用DMA接收的重载形式,AnyValue没有意义，仅用于编译器识别重载*/
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
		#if defined(SPI_TXDR_TXDR)/*H7系列*/
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
	
	/*使用DMA收发*/
	#if (SPI_USE_TX_DMA==1)&&(SPI_USE_RX_DMA==1)
	public:
	/*使用DMA收发的重载形式*/
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
		
		#if defined(SPI_TXDR_TXDR)/*H7系列*/
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
