#include "DL_H750.h"
#include "app_threadx.h"
#include "InitalTask.h"
#include "ReferDriver.h"


TX_THREAD LedThread;
TX_THREAD Test1Thread;
TX_SEMAPHORE LedSem;

uint8_t LedThreadStack[512]={0};
uint8_t Test1ThreadStack[256]={0};

TX_BYTE_POOL UARTPool;
SRAM_SET_D2 uint8_t UART_PoolBuf[8192]={0};//D2 串口内存池空间

extern void UART1ThreadFun(ULONG initial_input);
extern void UART2ThreadFun(ULONG initial_input);
extern void RemoterThreadFun(ULONG initial_input);
extern void INSThreadFun(ULONG initial_input);
extern void TemThreadFun (ULONG initial_input);
extern void CANThreadFun(ULONG initial_input);
extern void RoboCTRThreadFun(ULONG initial_input);
extern void CloseLoopThreadFun(ULONG initial_input);
	


extern TX_BYTE_POOL UARTPool;

extern TX_THREAD UART1Thread;
extern TX_SEMAPHORE UART1RXSem;
extern TX_SEMAPHORE UART1TXSem;

extern TX_THREAD UART2Thread;
extern TX_QUEUE  RefQue;
extern TX_SEMAPHORE  RefSem;

extern TX_THREAD RemoterThread;
extern TX_QUEUE  RemoterRXQue;

extern TX_THREAD 	CANThread;
extern TX_SEMAPHORE	CANSem;

extern TX_THREAD INSThread;
extern TX_SEMAPHORE IMUSem;
extern TX_THREAD TemThread;

extern TX_THREAD RoboCTRThread;
 
extern TX_THREAD CloseLoopThread;



extern uint8_t UART1ThreadStack[512];

extern uint8_t UART2ThreadStack[256];
extern uint8_t RefThreadStack[1024];
extern uint8_t RefQueueStack[32];

extern uint8_t RemoterThreadStack[512];
extern uint8_t RemoterQueueStack[32];

extern uint8_t CANThreadStack[4096];

extern uint8_t INSThreadStack[2048];
extern uint8_t TemThreadStack[512];

extern uint8_t RoboCTRThreadStack[2048];

extern uint8_t 	CloseLoopThreadStack[512];

float BatVal = 0.0f;
static void LedThreadFun(ULONG initial_input)
{
	ULONG ticker = 0;

	LL_TIM_EnableAllOutputs(TIM8);
	LL_TIM_CC_EnableChannel(TIM8,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM8,LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM8);
	LL_TIM_EnableAllOutputs(TIM4);
	LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableCounter(TIM4);
	
	/*电压检测配置使能*/
	LL_ADC_StartCalibration(ADC1,LL_ADC_CALIB_OFFSET_LINEARITY,LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC1))
	{tx_thread_sleep(100);}
	LL_OPAMP_Enable(OPAMP1);
	tx_thread_sleep(100);
	LL_ADC_Enable(ADC1);
	tx_thread_sleep(100);

	uint16_t RGColor=0;
	int16_t ADC_Val = 0;
	
	for(;;)
	{
		LL_ADC_REG_StartConversion(ADC1);
		
		/*No battery should not scream*/
		if((BatVal<22.5f)&&(BatVal>9.0f))
		{
			LL_TIM_OC_SetCompareCH1(TIM8,499);
			LL_TIM_OC_SetCompareCH2(TIM8,100);
			tx_thread_sleep(50);
			LL_TIM_OC_SetCompareCH1(TIM8,0);
			LL_TIM_OC_SetCompareCH2(TIM8,999);
			tx_thread_sleep(50);
		}
		else
		{
			/*按照红色计算*/
			if(BatVal>25.0f){RGColor=999;}
			else
			{
				RGColor = 400*(BatVal-22.5f);
			}
		
			/*绿色*/
			LL_TIM_OC_SetCompareCH3(TIM4,999-RGColor);
			LL_TIM_OC_SetCompareCH2(TIM8,RGColor);
			tx_thread_sleep(900);
			LL_TIM_OC_SetCompareCH3(TIM4,999);
			LL_TIM_OC_SetCompareCH2(TIM8,999);
			tx_thread_sleep(100);
		}
		
		ADC_Val = LL_ADC_REG_ReadConversionData16(ADC1);
		BatVal = 0.2f*BatVal + 0.0004431152f*(float)LL_ADC_REG_ReadConversionData16(ADC1);
	}
}

static void Test1ThreadFun(ULONG initial_input)
{
	ULONG ticker = 0;
	for(;;)
	{
		tx_semaphore_put(&LedSem);
		tx_thread_sleep(25);
	}
}


void Task_Init(void)
{
	LL_GPIO_SetOutputPin(EN_5V_GPIO_Port,EN_5V_Pin);
/**********内存池***********/
	tx_byte_pool_create(
		&UARTPool,
		(CHAR*)"UART_Pool",
		UART_PoolBuf,
		sizeof(UART_PoolBuf));

/**********信号量***********/	
	tx_semaphore_create(
		&CANSem,
		(CHAR*)"CANSem",
		0
		);

	tx_semaphore_create(
		&IMUSem,
		(CHAR*)"IMUSem",
		0
		);
/**********消息队列***********/
	/*DBUS*/
	tx_queue_create(
		&RemoterRXQue,
		(CHAR*)"REMOTERQUE",
		4, 
		RemoterQueueStack,
		sizeof(RemoterQueueStack));
		
/**********进程***********/

	tx_thread_create(
		&LedThread, 
		(CHAR*)"Led",
		LedThreadFun, 
		0x0000,
		LedThreadStack,
		sizeof(LedThreadStack),
		15,
		15,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
		
	tx_thread_create(
		&RemoterThread, 
		(CHAR*)"REMOTER",
		RemoterThreadFun, 
		0x0000,
		RemoterThreadStack,
		sizeof(RemoterThreadStack),
		5,
		5,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
			
				
	tx_thread_create(
		&CANThread, 
		(CHAR*)"CAN",
		CANThreadFun, 
		0x0000,
		CANThreadStack,
		sizeof(CANThreadStack),
		4,
		4,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
		
	tx_thread_create(
		&INSThread, 
		(CHAR*)"INS",
		INSThreadFun, 
		0x0000,
		INSThreadStack,
		sizeof(INSThreadStack),
		3,
		3,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);

	tx_thread_create(
		&TemThread, 
		(CHAR*)"INSTEM",
		TemThreadFun, 
		0x0000,
		TemThreadStack,
		sizeof(TemThreadStack),
		10,
		10,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
		
	tx_thread_create(
		&RoboCTRThread, 
		(CHAR*)"CONTROL",
		RoboCTRThreadFun, 
		0x0000,
		RoboCTRThreadStack,
		sizeof(RoboCTRThreadStack),
		6,
		6,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);

	tx_thread_create(
		&CloseLoopThread, 
		(CHAR*)"CLOSELOOP",
		CloseLoopThreadFun, 
		0x0000,
		CloseLoopThreadStack,
		sizeof(CloseLoopThreadStack),
		6,
		6,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
//	tx_thread_create(
//		&UART1Thread, 
//		(CHAR*)"UART1",
//		UART1ThreadFun, 
//		0x0000,
//		UART1ThreadStack,
//		sizeof(UART1ThreadStack),
//		5,
//		5,
//		TX_NO_TIME_SLICE,
//		TX_AUTO_START);
//	

	

	
//	/*REF*/	
//	tx_queue_create(
//		&RefQue,
//		(CHAR*)"REFQUE",
//		sizeof(ref_msg_t), 
//		RefQueueStack,
//		sizeof(RefQueueStack));
//	
//	tx_semaphore_create(
//		&RefSem,
//		(CHAR*)"RefSem",
//		0
//		);
//	
//	tx_thread_create(
//		&UART2Thread, 
//		(CHAR*)"UART2",
//		UART2ThreadFun, 
//		0x0000,
//		UART2ThreadStack,
//		sizeof(UART2ThreadStack),
//		5,
//		5,
//		TX_NO_TIME_SLICE,
//		TX_AUTO_START);
//	
//	/*CAN1*/
//	tx_semaphore_create(
//		&CAN1RXSem,
//		(CHAR*)"CAN1Sem",
//		0
//		);
//	
//	tx_thread_create(
//		&CAN1Thread, 
//		(CHAR*)"CAN1",
//		CAN1ThreadFun, 
//		0x0000,
//		CAN1ThreadStack,
//		sizeof(CAN1ThreadStack),
//		4,
//		4,
//		TX_NO_TIME_SLICE,
//		TX_AUTO_START);
		
}