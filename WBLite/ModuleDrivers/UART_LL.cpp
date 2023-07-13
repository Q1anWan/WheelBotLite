/*********************************************************************************
  *FileName:	UART_LL.cpp
  *Author:  	qianwan
  *Description: STM32 ���ڷ��ͻ���
  *Other:		ʹ�ú궨�忪��DMA�շ���DMA�ڴ����
				//////////////////////////////////////////////////////////////////
			**	UART_USE_TX_DMA == 1 ʱ��������DMA
				DMA������Ҫ��DMA��IRQ�е���Transmit_IRQ
				
				//////////////////////////////////////////////////////////////////
			**	UART_USE_RX_DMA == 1 ʱ��������DMA
				DMA������Ҫ��UART��IRQ�е���Recieve_IRQ

  *Version:  	2.2
  *Date:  		2023/05/12
  *Description:
				�޸��������͵�һ������

  *Version:  	2.1
  *Date:  		2023/04/10
  *Description:
				��״̬��, ������ȫ��
				
  *Version:  	2.0
  *Date:  		2023/04/05
  *Description:
				DMA���ջ����㷨��Ϊ���λ������
				����ͷ�ļ�ͼ�λ��������﷨
				
  *Version:  	1.1
  *Date:  		2023/04/01
  *Description:
				ȡ����Delay.c������
				�޶��жϻص�����
				��SPI����ͳһ���
				�ḻ���պ�������,����¶��������ַ
  
  *Version:  	1.0
  *Date:  		2022/06/27
  *Description: ������Ŀ
**********************************************************************************/
#include "UART_LL.h"
#include "string.h"
uint8_t cUART::Transmit(uint8_t *DT,uint16_t num,uint32_t OVT)
{
	/*�����*/
	if(this->TX_Statue!=STATUE_RDY)return 1;
	/*����*/
	this->TX_Statue = STATUE_BSY;
	
	volatile uint32_t wait_cnt = 0;
	uint16_t tx_len = 0;
	
	for (uint16_t i = 0; i < num; i++)
	{
		while (!LL_USART_IsActiveFlag_TXE(this->UART)) //�ȴ����ݷ������
		{
			this->Delay();
			/*��ʱ����*/
			if (wait_cnt++ > OVT)
			{
				return 1;
				this->Transmit_Length = tx_len;
				this->TX_Statue = STATUE_RDY;
			}
		}
 		LL_USART_TransmitData8(this->UART, DT[i]); //����8λ����
		tx_len++;
	}
	
	this->Transmit_Length = tx_len;
	wait_cnt = 0;
	while (!LL_USART_IsActiveFlag_TXE(this->UART)) //�ȴ����ݷ������
	{
		this->Delay();
		if (wait_cnt++ > OVT)
		{
			return 1;
			this->TX_Statue = STATUE_RDY;
		}
	}
	/*����*/
	this->TX_Statue = STATUE_RDY;
	return 0;
}

uint8_t cUART::Recieve(uint8_t *DT,uint16_t num,uint32_t OVT)
{
	/*�����*/
	if(this->RX_Statue!=STATUE_RDY)return 1;
	/*����*/
	this->RX_Statue = STATUE_BSY;
	
	/*��¼���ܳ���*/
	this->TargetRecieve_Length = num;
	
	uint16_t wait_cnt = 0;
	uint16_t i = 0;
	
	/*����ѭ������*/
	for (; i < num; i++)
	{
		/*�ȴ����ݲ���¼��ʱ*/
		while(!LL_USART_IsActiveFlag_RXNE(this->UART))
		{ 
			this->Delay();
			/*��ʱ����*/
			if(wait_cnt++>OVT)
			{
				this->Recieve_Length = i;
				this->RX_Statue = STATUE_RDY;
				return 1;
			}
		}
		
		/*RBNEʱ��������,�˲��������RBNE��־*/
		DT[i]=LL_USART_ReceiveData8(this->UART);
		wait_cnt = 0;
	}
	
	this->RX_Statue = STATUE_RDY;
	this->Recieve_Length = i;
	return 0;
}

/*
	��ʹ��DMA�ĳ�ʼ������
*/
uint8_t cUART::UART_Init(USART_TypeDef *UART)
{
	this->UART = UART;
	
	this->TX_Statue = STATUE_RDY;
	this->RX_Statue = STATUE_RDY;
	
	LL_USART_Enable(this->UART);
	return 0;
}

/*ʹ��DMA����*/
#if (UART_USE_TX_DMA==1)
/*
	ʹ��DMA�����͵�������ʽ
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
	/*״̬���*/
	if(this->TX_Statue!=STATUE_RDY){return 1;}
	if( (data == 0) || (num == 0) ){return 1;}
	
	/*����*/
	this->TX_Statue = STATUE_BSY;
	
	/*����DMA*/	
	LL_DMA_SetMemoryAddress(this->DMAt.DMAx, this->DMAt.CH, (uint32_t)data);
	LL_DMA_SetDataLength(this->DMAt.DMAx, this->DMAt.CH, num);
	
	/*����*/
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

/*ʹ��DMA����������*/
#if (UART_USE_RX_DMA==1)
/*
	ʹ��DMA���յ�������ʽ
	isMEMCPY == 1,���պ��������������ݿ����������ָ��
	isMEMCPY == 0,���պ�����������ָ�븳�������ָ��
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
	�˺�����������Ϊ����ֱ�ӿ�����ָ����ַ,��������Ϊ�����������ַ,ͨ��isMEMCPY��־λ
	������ָ����ַ�ǳ���Ľ��ܷ���,�����������ַ������������DMA����
	data:����ָ��,�������ݿ���ʱ,��������ָ���ָ�벢ִ��ǿ������׼�� (uint8_t**)&pArray
				 ���������ݿ���ʱ,����ָ���ָ�� &pPoint
	num:�������
*/
uint8_t cUART::Recieve_DMA(uint8_t* *data,uint16_t num)
{
	/*״̬���*/
	if(this->RX_Statue!=STATUE_RDY){return 1;}
	if(num == 0){return 1;}
	/*����*/
	this->RX_Statue = STATUE_BSY;
	
	/*���ָ��*/
	this->pRX_DataPoint = data;
	/*��������*/
	this->TargetRecieve_Length = (num>this->RX_BUF_Size)?this->RX_BUF_Size:num;
	/*������������,��������ͷ��ʼ*/
	if(this->TargetRecieve_Length > this->RX_BUF_RemainSize)
	{this->RX_BUF_T = this->RX_BUF;}
	
	/*����DMA����*/
	LL_DMA_SetDataLength(this->DMAr.DMAx, this->DMAr.CH, this->TargetRecieve_Length);
	/*��������ַ*/
	LL_DMA_SetMemoryAddress(this->DMAr.DMAx, this->DMAr.CH, (uint32_t)this->RX_BUF_T);

	/*����жϱ�־λ*/
	LL_USART_ClearFlag_IDLE(this->UART); 
	/*�����־λ*/
	LL_USART_ReceiveData8(this->UART);
	/*���������ж�*/
	LL_USART_EnableIT_IDLE(this->UART);
	
	/*���DMA��־*/
	this->DMAr.ClearFlag_TC();
	/*����DMAͨ��*/
	this->DMAr.EnableCH();
	
	return 0;
}
/*���ؽ������ݳ���*/
uint16_t cUART::IRQ_Rx(void)
{
	/*�رմ���*/
	this->DMAr.DisableCH();
	/*�����־λ*/
	this->DMAr.ClearFlag_TC();
	/*�ر��ж�*/
	LL_USART_DisableIT_IDLE(this->UART);
	/*�����־λ*/
	LL_USART_ClearFlag_IDLE(this->UART);
	LL_USART_ClearFlag_ORE(this->UART);
	/*���δ�����������*/
	this->Recieve_Length = this->TargetRecieve_Length - LL_DMA_GetDataLength(this->DMAr.DMAx, this->DMAr.CH); 
	
	/*�������ݻ�������˶����ݻ�����ָ��*/
	if(this->isMEMCPY)
	{memcpy((uint8_t *)this->pRX_DataPoint, this->RX_BUF_T, this->Recieve_Length);}
	else
	{*pRX_DataPoint= this->RX_BUF_T;}
	
	/*���»�������βָ��*/
	this->RX_BUF_T = this->RX_BUF_T + this->Recieve_Length;
	/*���»�����ʣ������*/
	this->RX_BUF_RemainSize = this->cUART::RX_BUF + this->RX_BUF_Size - RX_BUF_T;
	
	/*����*/
	this->RX_Statue = STATUE_RDY;
	return this->Recieve_Length;
}
#endif

/*ͬʱʹ��DMA���պͷ���*/
#if (UART_USE_TX_DMA==1)&&(UART_USE_RX_DMA==1)
/*ʹ��DMA���պͷ��͵�������ʽ*/
uint8_t cUART::UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAr, uint32_t DMA_CHr, uint8_t isMEMCPY,
								   DMA_TypeDef *DMAt, uint32_t DMA_CHt)
{
	this->UART_Init(UART,DMAr,DMA_CHr,isMEMCPY);
	this->UART_Init(UART,DMAt,DMA_CHt);
	return 0;
}
#endif