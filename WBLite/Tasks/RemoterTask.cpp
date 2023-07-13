#include "RemoterTask.h"
#include "arm_math.h"

extern TX_BYTE_POOL UARTPool;

TX_THREAD RemoterThread;
TX_QUEUE  RemoterRXQue;
uint8_t 		RemoterThreadStack[512]={0};//DTCM UART3栈空间
SRAM_SET_D2		uint8_t RemoterQueueStack[32]={0};  //DTCM UART3消息队列栈空间

/*遥控器*/
cRemoter *Msg_Remoter;
RC_Ctl_t rc_data_buf;//DTCM 遥控器数据区域

void RemoterThreadFun(ULONG initial_input)
{
	Msg_Remoter = new cRemoter;
	uint8_t *precbuf = 0;
	
	/*指定接收缓冲区*/
	tx_byte_allocate(&UARTPool, (VOID **)&precbuf, BUS_LEN*5, TX_NO_WAIT);
	Msg_Remoter->UART_BufferConfig(precbuf,BUS_LEN*5);
	
	/*指定RC数据区域*/
	Msg_Remoter->ConfigRemoter(&rc_data_buf);
	
	/*初始化收发程序*/
	Msg_Remoter->UART_Init(UART4,DMA1,LL_DMA_STREAM_0,0);
	
	/*开启接收*/
	Msg_Remoter->Recieve_DMA(&Msg_Remoter->pbuf,BUS_LEN);
	
	/*清除接收机上电异常数据*/
	tx_thread_sleep(100);
	tx_queue_flush(&RemoterRXQue);
	
	/*从消息队列中接收的地址*/
	uint8_t *pbuf;
	
	/*暂存*/
	int16_t CHBuf[7]={0};
	
	int16_t rmtmid	= (BUS_MAX + BUS_MIN)/2;
	float rmthalfrev = 2.0f/(float)(BUS_MAX - BUS_MIN); 
	int16_t swcal0 = (BUS_MAX - BUS_MIN)/2;
	int16_t swcal1 = swcal0 - BUS_MIN;
	for(;;)
	{
		/*获取遥控器接受信号量*/
		if(tx_queue_receive(&RemoterRXQue, (VOID*)&pbuf, 30)==0x0A)
		{
			//超时,遥控器离线
			Msg_Remoter->IsRCOffline = 1;
			tx_queue_receive(&RemoterRXQue, (VOID*)&pbuf, TX_WAIT_FOREVER);
			//超时,遥控器连接正常
			Msg_Remoter->IsRCOffline = 0;
		}
		
		/*清理CACHE*/
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)pbuf,BUS_LEN);
		
		/*缓冲赋值*/
		CHBuf[0] = 		(((int16_t)pbuf[1] 		  | ((int16_t)pbuf[2] << 8)) & 0x07FF)								- rmtmid; 
		CHBuf[1] = 		((((int16_t)pbuf[2]>> 3)  | ((int16_t)pbuf[3] << 5)) & 0x07FF)								- rmtmid;
		CHBuf[2] = 		((((int16_t)pbuf[3] >> 6) | ((int16_t)pbuf[4] << 2) | ((int16_t)pbuf[5] << 10)) & 0x07FF)	- rmtmid;
		CHBuf[3] = 		((((int16_t)pbuf[5] >> 1) | ((int16_t)pbuf[6] << 7)) & 0x07FF)								- rmtmid;
		CHBuf[4] = 		((((int16_t)pbuf[6] >> 4) | ((int16_t)pbuf[7] << 4)) & 0x07FF)								;
		CHBuf[5] =		((((int16_t)pbuf[7] >> 7) | ((int16_t)pbuf[8] << 1) | ((int16_t)pbuf[9] << 9)) & 0x07FF)	;
		CHBuf[6] = 		((((int16_t)pbuf[9] >> 2) | ((int16_t)pbuf[10]<< 6)) & 0x07FF)								- rmtmid;
		
		if(abs(CHBuf[0])<BUS_DEATHZOON)
		{CHBuf[0] = 0;}
		if(abs(CHBuf[1])<BUS_DEATHZOON)
		{CHBuf[1] = 0;}
		if(abs(CHBuf[2])<BUS_DEATHZOON)
		{CHBuf[2] = 0;}
		if(abs(CHBuf[3])<BUS_DEATHZOON)
		{CHBuf[3] = 0;}
		
		
		Msg_Remoter->RC_Data->rmt.CH0 = 		rmthalfrev*(float)(CHBuf[0]);
		Msg_Remoter->RC_Data->rmt.CH3 = 		rmthalfrev*(float)(CHBuf[1]);
		Msg_Remoter->RC_Data->rmt.CH1 = 		rmthalfrev*(float)(CHBuf[2]);
		Msg_Remoter->RC_Data->rmt.CH2 = 		rmthalfrev*(float)(CHBuf[3]);
		Msg_Remoter->RC_Data->rmt.SW1 = 		(CHBuf[4]+swcal1)/swcal0;
		Msg_Remoter->RC_Data->rmt.SW2 =			(CHBuf[5]+swcal1)/swcal0;
		Msg_Remoter->RC_Data->rmt.WHEEL = 		rmthalfrev*(float)(CHBuf[6]);
				
//		/*遥控器赋值*/
//		Msg_Remoter->RC_Data->rc.ch0 = (((int16_t)pbuf[0] | ((int16_t)pbuf[1] << 8)) & 0x07FF) - 1024; 
//		Msg_Remoter->RC_Data->rc.ch1 = ((((int16_t)pbuf[1] >> 3) | ((int16_t)pbuf[2] << 5)) & 0x07FF) - 1024;
//		Msg_Remoter->RC_Data->rc.ch2 = ((((int16_t)pbuf[2] >> 6) | ((int16_t)pbuf[3] << 2) | ((int16_t)pbuf[4] << 10)) & 0x07FF) - 1024;
//		Msg_Remoter->RC_Data->rc.ch3 = ((((int16_t)pbuf[4] >> 1) | ((int16_t)pbuf[5]<<7)) & 0x07FF) - 1024;

//		Msg_Remoter->RC_Data->rc.s1 = ((pbuf[5] >> 4) & 0x000C) >> 2;
//		Msg_Remoter->RC_Data->rc.s2 = ((pbuf[5] >> 4) & 0x0003);
//		Msg_Remoter->RC_Data->rc.wheel = ((pbuf[16] | (pbuf[17] << 8)) & 0x07FF) - 1024;
//		
//		Msg_Remoter->RC_Data->mouse.x = ((int16_t)pbuf[6]) | ((int16_t)pbuf[7] << 8);
//		Msg_Remoter->RC_Data->mouse.y = ((int16_t)pbuf[8]) | ((int16_t)pbuf[9] << 8);
//		Msg_Remoter->RC_Data->mouse.z = ((int16_t)pbuf[10]) | ((int16_t)pbuf[11] << 8); 
//		Msg_Remoter->RC_Data->mouse.press_l = pbuf[12];
//		Msg_Remoter->RC_Data->mouse.press_r = pbuf[13];
//		
//		Msg_Remoter->RC_Data->key.v = pbuf[14];
		Msg_Remoter->IsRCOffline = pbuf[23];
	}
}
void UART4_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(UART4))
	{
		Msg_Remoter->IRQ_Rx();
		tx_queue_send(&RemoterRXQue, (VOID*)&Msg_Remoter->pbuf, TX_NO_WAIT);
		Msg_Remoter->Recieve_DMA(&Msg_Remoter->pbuf,BUS_LEN);
	}

}
void DMA1_Stream0_IRQHandler(void)
{

}
