/*********************************************************************************
  *FileName:	UART_LL.cpp
  *Author:  	qianwan
  *Description: STM32 串口发送基类
  *Other:		使用宏定义开启DMA收发与DMA内存搬运
				//////////////////////////////////////////////////////////////////
			**	UART_USE_TX_DMA == 1 时开启发送DMA
				DMA发送需要在DMA的IRQ中调用Transmit_IRQ
				
				//////////////////////////////////////////////////////////////////
			**	UART_USE_RX_DMA == 1 时开启接收DMA
				DMA接收需要在UART的IRQ中调用Recieve_IRQ

  *Version:  	2.2
  *Date:  		2023/05/12
  *Description:
				修复阻塞发送的一处问题

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
#include "UART_LL.h"
#include "string.h"
uint8_t cUART::Transmit(uint8_t *DT,uint16_t num,uint32_t OVT)
{
	/*检查锁*/
	if(this->TX_Statue!=STATUE_RDY)return 1;
	/*上锁*/
	this->TX_Statue = STATUE_BSY;
	
	volatile uint32_t wait_cnt = 0;
	uint16_t tx_len = 0;
	
	for (uint16_t i = 0; i < num; i++)
	{
		while (!LL_USART_IsActiveFlag_TXE(this->UART)) //等待数据发送完成
		{
			this->Delay();
			/*超时处理*/
			if (wait_cnt++ > OVT)
			{
				return 1;
				this->Transmit_Length = tx_len;
				this->TX_Statue = STATUE_RDY;
			}
		}
 		LL_USART_TransmitData8(this->UART, DT[i]); //发送8位数据
		tx_len++;
	}
	
	this->Transmit_Length = tx_len;
	wait_cnt = 0;
	while (!LL_USART_IsActiveFlag_TXE(this->UART)) //等待数据发送完成
	{
		this->Delay();
		if (wait_cnt++ > OVT)
		{
			return 1;
			this->TX_Statue = STATUE_RDY;
		}
	}
	/*解锁*/
	this->TX_Statue = STATUE_RDY;
	return 0;
}

uint8_t cUART::Recieve(uint8_t *DT,uint16_t num,uint32_t OVT)
{
	/*检查锁*/
	if(this->RX_Statue!=STATUE_RDY)return 1;
	/*上锁*/
	this->RX_Statue = STATUE_BSY;
	
	/*记录接受长度*/
	this->TargetRecieve_Length = num;
	
	uint16_t wait_cnt = 0;
	uint16_t i = 0;
	
	/*数据循环接受*/
	for (; i < num; i++)
	{
		/*等待数据并记录超时*/
		while(!LL_USART_IsActiveFlag_RXNE(this->UART))
		{ 
			this->Delay();
			/*超时处理*/
			if(wait_cnt++>OVT)
			{
				this->Recieve_Length = i;
				this->RX_Statue = STATUE_RDY;
				return 1;
			}
		}
		
		/*RBNE时接收数据,此操作将清除RBNE标志*/
		DT[i]=LL_USART_ReceiveData8(this->UART);
		wait_cnt = 0;
	}
	
	this->RX_Statue = STATUE_RDY;
	this->Recieve_Length = i;
	return 0;
}

/*
	不使用DMA的初始化函数
*/
uint8_t cUART::UART_Init(USART_TypeDef *UART)
{
	this->UART = UART;
	
	this->TX_Statue = STATUE_RDY;
	this->RX_Statue = STATUE_RDY;
	
	LL_USART_Enable(this->UART);
	return 0;
}

/*使用DMA发送*/
#if (UART_USE_TX_DMA==1)
/*
	使用DMA单发送的重载形式
*/
uint8_t cUART::UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAt, uint32_t DMA_CHt)
{
	this->DMAt.Config(DMAt,DMA_CHt);
	this->UART = UART;
	
	this->TX_Statue = STATUE_RDY;
	this->RX_Statue = STATUE_RDY;
	
	LL_USART_EnableDMAReq_TX(this->UART);
	#if defined(USART_TDR_TDR)
	LL_DMA_SetPeriphAddress(this->DMAt.DMAx, this->DMAt.CH, (uint32_t)&this->UART->TDR);
	#else
	LL_DMA_SetPeriphAddress(this->DMAt.DMAx, this->DMAt.CH, (uint32_t)&this->UART->DR);
	#endif
	
	this->DMAt.ClearFlag_TC();
	this->DMAt.ClearFlag_HT();
	this->DMAt.ClearFlag_TE();
	
	LL_DMA_EnableIT_TC(this->DMAt.DMAx,this->DMAt.CH);
	LL_USART_Enable(this->UART);
	
	return 0;
}

uint8_t cUART::Transmit_DMA(uint8_t *data,uint16_t num)
{
	/*状态检查*/
	if(this->TX_Statue!=STATUE_RDY){return 1;}
	if( (data == 0) || (num == 0) ){return 1;}
	
	/*上锁*/
	this->TX_Statue = STATUE_BSY;
	
	/*配置DMA*/	
	LL_DMA_SetMemoryAddress(this->DMAt.DMAx, this->DMAt.CH, (uint32_t)data);
	LL_DMA_SetDataLength(this->DMAt.DMAx, this->DMAt.CH, num);
	
	/*发送*/
	this->DMAt.ClearFlag_TC();
	this->DMAt.EnableCH();
	
	return 0;
}

uint8_t cUART::IRQ_Tx(void)
{
	this->DMAt.DisableCH();
	this->DMAt.ClearFlag_TC();
	this->TX_Statue = STATUE_RDY;
	return 0;
}

#endif

/*使用DMA不定长接收*/
#if (UART_USE_RX_DMA==1)
/*
	使用DMA接收的重载形式
	isMEMCPY == 1,接收函数将缓冲区数据拷贝到传入的指针
	isMEMCPY == 0,接收函数将缓冲区指针赋给传入的指针
*/
uint8_t cUART::UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAr, uint32_t DMA_CHr, uint8_t isMEMCPY)
{
	this->DMAr.Config(DMAr,DMA_CHr);
	this->UART = UART;
	
	this->isMEMCPY = isMEMCPY;
	
	this->TX_Statue = STATUE_RDY;
	this->RX_Statue = STATUE_RDY;
	
	LL_USART_EnableDMAReq_RX(this->UART);
	#if defined(USART_RDR_RDR)
	LL_DMA_SetPeriphAddress(this->DMAr.DMAx, this->DMAr.CH, (uint32_t)&this->UART->RDR);
	#else
	LL_DMA_SetPeriphAddress(this->DMAr.DMAx, this->DMAr.CH, (uint32_t)&this->UART->DR);
	#endif
	
	if(this->RX_BUF == 0)
	{
		#ifdef __RT_THREAD_H__
		this->RX_BUF = (uint8_t*)rt_malloc(sizeof(uint8_t)*this->RX_BUF_Size);
		#else
		this->RX_BUF = (uint8_t*)malloc(sizeof(uint8_t)*this->RX_BUF_Size);
		#endif
		
		if(this->RX_BUF==0)
		{return 1;}
	}

	this->cUART::RX_BUF_T = this->RX_BUF;
	this->RX_BUF_RemainSize = this->cUART::RX_BUF + this->RX_BUF_Size - RX_BUF_T;
	
	this->DMAr.ClearFlag_TC();
	this->DMAr.ClearFlag_HT();
	this->DMAr.ClearFlag_TE();
	
	LL_USART_ClearFlag_IDLE(this->UART);
	LL_USART_Enable(this->UART);
	return 0;
}
uint8_t cUART::UART_BufferConfig(uint8_t *FIFOData,uint16_t Size)
{
	this->cUART::RX_BUF = FIFOData;
	this->cUART::RX_BUF_Size = Size;
	return 0;
}
/*
	此函数可以配置为数据直接拷贝到指定地址,或者配置为输出缓冲区地址,通过isMEMCPY标志位
	拷贝到指定地址是常规的接受方法,输出缓冲区地址用于适配两段DMA操作
	data:二级指针,启用数据拷贝时,传入数组指针的指针并执行强制类型准换 (uint8_t**)&pArray
				 不启用数据拷贝时,传入指针的指针 &pPoint
	num:最长接收数
*/
uint8_t cUART::Recieve_DMA(uint8_t* *data,uint16_t num)
{
	/*状态检查*/
	if(this->RX_Statue!=STATUE_RDY){return 1;}
	if(num == 0){return 1;}
	/*上锁*/
	this->RX_Statue = STATUE_BSY;
	
	/*存好指针*/
	this->pRX_DataPoint = data;
	/*长度设置*/
	this->TargetRecieve_Length = (num>this->RX_BUF_Size)?this->RX_BUF_Size:num;
	/*储存容量不足,缓冲区从头开始*/
	if(this->TargetRecieve_Length > this->RX_BUF_RemainSize)
	{this->RX_BUF_T = this->RX_BUF;}
	
	/*设置DMA长度*/
	LL_DMA_SetDataLength(this->DMAr.DMAx, this->DMAr.CH, this->TargetRecieve_Length);
	/*缓冲区地址*/
	LL_DMA_SetMemoryAddress(this->DMAr.DMAx, this->DMAr.CH, (uint32_t)this->RX_BUF_T);

	/*清除中断标志位*/
	LL_USART_ClearFlag_IDLE(this->UART); 
	/*清除标志位*/
	LL_USART_ReceiveData8(this->UART);
	/*开启空闲中断*/
	LL_USART_EnableIT_IDLE(this->UART);
	
	/*清除DMA标志*/
	this->DMAr.ClearFlag_TC();
	/*开启DMA通道*/
	this->DMAr.EnableCH();
	
	return 0;
}
/*返回接收数据长度*/
uint16_t cUART::IRQ_Rx(void)
{
	/*关闭传输*/
	this->DMAr.DisableCH();
	/*清除标志位*/
	this->DMAr.ClearFlag_TC();
	/*关闭中断*/
	LL_USART_DisableIT_IDLE(this->UART);
	/*清除标志位*/
	LL_USART_ClearFlag_IDLE(this->UART);
	LL_USART_ClearFlag_ORE(this->UART);
	/*本次传输数据数量*/
	this->Recieve_Length = this->TargetRecieve_Length - LL_DMA_GetDataLength(this->DMAr.DMAx, this->DMAr.CH); 
	
	/*拷贝数据或者输出此段数据缓冲区指针*/
	if(this->isMEMCPY)
	{memcpy((uint8_t *)this->pRX_DataPoint, this->RX_BUF_T, this->Recieve_Length);}
	else
	{*pRX_DataPoint= this->RX_BUF_T;}
	
	/*更新缓冲区队尾指针*/
	this->RX_BUF_T = this->RX_BUF_T + this->Recieve_Length;
	/*更新缓冲区剩余容量*/
	this->RX_BUF_RemainSize = this->cUART::RX_BUF + this->RX_BUF_Size - RX_BUF_T;
	
	/*解锁*/
	this->RX_Statue = STATUE_RDY;
	return this->Recieve_Length;
}
#endif

/*同时使用DMA接收和发送*/
#if (UART_USE_TX_DMA==1)&&(UART_USE_RX_DMA==1)
/*使用DMA接收和发送的重载形式*/
uint8_t cUART::UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAr, uint32_t DMA_CHr, uint8_t isMEMCPY,
								   DMA_TypeDef *DMAt, uint32_t DMA_CHt)
{
	this->UART_Init(UART,DMAr,DMA_CHr,isMEMCPY);
	this->UART_Init(UART,DMAt,DMA_CHt);
	return 0;
}
#endif