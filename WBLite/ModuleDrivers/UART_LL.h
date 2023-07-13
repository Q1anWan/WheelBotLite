/*********************************************************************************
  *FileName:	UART_LL.h
  *Author:  	qianwan
  *Description: STM32 串口发送基类
  *Other:		使用宏定义开启DMA收发与DMA内存搬运
				//////////////////////////////////////////////////////////////////
			**	UART_USE_TX_DMA == 1 时开启发送DMA
				DMA发送需要在DMA的IRQ中调用Transmit_IRQ
				
				//////////////////////////////////////////////////////////////////
			**	UART_USE_RX_DMA == 1 时开启接收DMA
				DMA接收需要在UART的IRQ中调用Recieve_IRQ

  *Version:  	2.1
  *Date:  		2023/04/10
  *Description:
				加状态锁, 提升安全性
				
  *Version:  	2.0
  *Date:  		2023/04/05
  *Description:
				DMA接收缓冲算法改为环形缓冲队列
				增加头文件图形化配置向导语法
				
  *Version:  	1.1
  *Date:  		2023/04/01
  *Description:
				取消对Delay.c的依赖
				修订中断回调函数
				与SPI函数统一风格
				丰富接收函数功能,允许暴露缓冲区地址
  
  *Version:  	1.0
  *Date:  		2022/06/27
  *Description: 创建项目
**********************************************************************************/
#ifndef UART_LL_H
#define UART_LL_H
#include "main.h"
#include "string.h"

// <<< Use Configuration Wizard in Context Menu >>>
/*******开启DMA收发*****s***/

// <o>阻塞收发延时函数基准空周期数
//  <i>建议使用CPU主频/1MHz
#define UART_DELAY_BASE  240

// <c1>启用串口DMA发送
//  <i>启用串口DMA发送
#define UART_USE_TX_DMA 1
// </c>


// <e>启用串口DMA接收
// <i> 启用串口DMA接收
#define UART_USE_RX_DMA 1
	// <o>默认缓冲区长度
	//  <i>默认缓冲区的长度
	//  <i>Default: 128  (128Byte)
	#define RX_BUF_LEN  128
	#if (UART_USE_RX_DMA==0)
	#undef RX_BUF_LEN
	#endif
// </e>

// <<< end of configuration section >>>

/********1开**0关***********/
#if (UART_USE_RX_DMA)||(UART_USE_TX_DMA)
#include "DMAFlag_LL.h"
#endif
/**************************/

#ifdef __cplusplus


/*
	串口基类
	不使用DMA时使用此类实体化
	具有阻塞接收、发送功能

	使用前先执行Init函数
*/
class cUART
{
	protected:
	
	/*UART指针*/
	USART_TypeDef *UART;
	
	/*延时函数，用户*/
	virtual inline void Delay(void)
	{for(volatile uint16_t i=0;i<UART_DELAY_BASE;i++){__NOP();}}
	
	/*状态量, 构成锁, 确保线程安全*/
	enum eStatue
	{
		STATUE_RDY = 0,
		STATUE_BSY = 1
	};
	eStatue RX_Statue = STATUE_RDY;
	eStatue TX_Statue = STATUE_RDY;
	
	uint16_t Transmit_Length=0;
	uint16_t TargetRecieve_Length =0;
	uint16_t Recieve_Length=0;
	
	public:
	
	uint8_t UART_Init(USART_TypeDef *UART);
	uint8_t Transmit(uint8_t *DT,uint16_t num,uint32_t OVT);
	uint8_t Recieve(uint8_t *DT,uint16_t num,uint32_t OVT);
	

	/*返回实际发送长度*/
	inline uint16_t GetTL(void){return Transmit_Length;}
	/*返回期望接收长度*/
	inline uint16_t GetTargetRL(void){return TargetRecieve_Length;}
	/*返回实际接收长度*/
	inline uint16_t GetRL(void){return Recieve_Length;}

	/*使用DMA发送*/
	#if (UART_USE_TX_DMA==1)
	protected:
	cDMA DMAt;
	public:
	/*使用DMA单发送的重载形式*/
	uint8_t UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAt, uint32_t DMA_CHt);
	uint8_t Transmit_DMA(uint8_t *data,uint16_t num);
	uint8_t IRQ_Tx(void);
	#endif
	
	/*使用DMA不定长接收*/
	#if (UART_USE_TX_DMA==1)
	protected:
	cDMA DMAr;
	uint8_t  *RX_BUF = 0;				  //FIFO首地址
	uint8_t  *RX_BUF_T = 0;			  //当前FIFO地址
	uint16_t RX_BUF_Size = RX_BUF_LEN; //FIFO容量
	uint16_t RX_BUF_RemainSize = 0;	  //FIFO剩余容量
	
	uint8_t* *pRX_DataPoint;
	uint8_t isMEMCPY = 1;
	public:
	/*
		使用DMA接收的重载形式
		isMEMCPY == 1,接收函数将缓冲区数据拷贝到传入的指针
		isMEMCPY == 0,接收函数将缓冲区指针赋给传入的指针
	*/
	uint8_t UART_BufferConfig(uint8_t *FIFOData,uint16_t Size);
	uint8_t UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAr, uint32_t DMA_CHr, uint8_t isMEMCPY);
	uint8_t Recieve_DMA(uint8_t* *data,uint16_t num);
	uint16_t IRQ_Rx(void);
	#endif
	
	/*同时使用DMA接收和发送*/
	#if (UART_USE_TX_DMA==1)&&(UART_USE_RX_DMA==1)
	public:
	/*使用DMA接收的重载形式*/
	uint8_t UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAr, uint32_t DMA_CHr, uint8_t isMEMCPY,
									   DMA_TypeDef *DMAt, uint32_t DMA_CHt);
	#endif
};

extern "C"
{
	/*C Code Begin*/
	/*C Code End*/
}
#endif
#endif