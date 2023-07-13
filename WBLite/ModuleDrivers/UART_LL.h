/*********************************************************************************
  *FileName:	UART_LL.h
  *Author:  	qianwan
  *Description: STM32 ���ڷ��ͻ���
  *Other:		ʹ�ú궨�忪��DMA�շ���DMA�ڴ����
				//////////////////////////////////////////////////////////////////
			**	UART_USE_TX_DMA == 1 ʱ��������DMA
				DMA������Ҫ��DMA��IRQ�е���Transmit_IRQ
				
				//////////////////////////////////////////////////////////////////
			**	UART_USE_RX_DMA == 1 ʱ��������DMA
				DMA������Ҫ��UART��IRQ�е���Recieve_IRQ

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
#ifndef UART_LL_H
#define UART_LL_H
#include "main.h"
#include "string.h"

// <<< Use Configuration Wizard in Context Menu >>>
/*******����DMA�շ�*****s***/

// <o>�����շ���ʱ������׼��������
//  <i>����ʹ��CPU��Ƶ/1MHz
#define UART_DELAY_BASE  240

// <c1>���ô���DMA����
//  <i>���ô���DMA����
#define UART_USE_TX_DMA 1
// </c>


// <e>���ô���DMA����
// <i> ���ô���DMA����
#define UART_USE_RX_DMA 1
	// <o>Ĭ�ϻ���������
	//  <i>Ĭ�ϻ������ĳ���
	//  <i>Default: 128  (128Byte)
	#define RX_BUF_LEN  128
	#if (UART_USE_RX_DMA==0)
	#undef RX_BUF_LEN
	#endif
// </e>

// <<< end of configuration section >>>

/********1��**0��***********/
#if (UART_USE_RX_DMA)||(UART_USE_TX_DMA)
#include "DMAFlag_LL.h"
#endif
/**************************/

#ifdef __cplusplus


/*
	���ڻ���
	��ʹ��DMAʱʹ�ô���ʵ�廯
	�����������ա����͹���

	ʹ��ǰ��ִ��Init����
*/
class cUART
{
	protected:
	
	/*UARTָ��*/
	USART_TypeDef *UART;
	
	/*��ʱ�������û�*/
	virtual inline void Delay(void)
	{for(volatile uint16_t i=0;i<UART_DELAY_BASE;i++){__NOP();}}
	
	/*״̬��, ������, ȷ���̰߳�ȫ*/
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
	

	/*����ʵ�ʷ��ͳ���*/
	inline uint16_t GetTL(void){return Transmit_Length;}
	/*�����������ճ���*/
	inline uint16_t GetTargetRL(void){return TargetRecieve_Length;}
	/*����ʵ�ʽ��ճ���*/
	inline uint16_t GetRL(void){return Recieve_Length;}

	/*ʹ��DMA����*/
	#if (UART_USE_TX_DMA==1)
	protected:
	cDMA DMAt;
	public:
	/*ʹ��DMA�����͵�������ʽ*/
	uint8_t UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAt, uint32_t DMA_CHt);
	uint8_t Transmit_DMA(uint8_t *data,uint16_t num);
	uint8_t IRQ_Tx(void);
	#endif
	
	/*ʹ��DMA����������*/
	#if (UART_USE_TX_DMA==1)
	protected:
	cDMA DMAr;
	uint8_t  *RX_BUF = 0;				  //FIFO�׵�ַ
	uint8_t  *RX_BUF_T = 0;			  //��ǰFIFO��ַ
	uint16_t RX_BUF_Size = RX_BUF_LEN; //FIFO����
	uint16_t RX_BUF_RemainSize = 0;	  //FIFOʣ������
	
	uint8_t* *pRX_DataPoint;
	uint8_t isMEMCPY = 1;
	public:
	/*
		ʹ��DMA���յ�������ʽ
		isMEMCPY == 1,���պ��������������ݿ����������ָ��
		isMEMCPY == 0,���պ�����������ָ�븳�������ָ��
	*/
	uint8_t UART_BufferConfig(uint8_t *FIFOData,uint16_t Size);
	uint8_t UART_Init(USART_TypeDef *UART, DMA_TypeDef *DMAr, uint32_t DMA_CHr, uint8_t isMEMCPY);
	uint8_t Recieve_DMA(uint8_t* *data,uint16_t num);
	uint16_t IRQ_Rx(void);
	#endif
	
	/*ͬʱʹ��DMA���պͷ���*/
	#if (UART_USE_TX_DMA==1)&&(UART_USE_RX_DMA==1)
	public:
	/*ʹ��DMA���յ�������ʽ*/
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