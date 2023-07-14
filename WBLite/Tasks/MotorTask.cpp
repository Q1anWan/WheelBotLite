#include "MotorTask.h"
#include "DMDriver.h"
#include "RemoterTask.h"


TX_THREAD		CANThread;
TX_SEMAPHORE	CANSem;
uint8_t			CANThreadStack[4096]={0};

cMotorUnit *MotorUnit;
extern cRemoter *Msg_Remoter;

ULONG timess=0;
void CANThreadFun(ULONG initial_input)
{
	CANFilterConfig();
	cMotorUnit MotorUnitt;
	MotorUnit = &MotorUnitt;
	for(uint8_t i=0;i<6;i++)
	{MotorUnit->Motor[i].SetID(&hfdcan2,0x101+i);}

	tx_thread_sleep(5000);
	
	while(tx_semaphore_get(&CANSem,0)==TX_SUCCESS);
	MotorUnit->Motor[0].EnableMotor();tx_semaphore_get(&CANSem,5);tx_thread_sleep(1);
	MotorUnit->Motor[1].EnableMotor();tx_semaphore_get(&CANSem,5);tx_thread_sleep(1);
	MotorUnit->Motor[2].EnableMotor();tx_semaphore_get(&CANSem,5);tx_thread_sleep(1);
	MotorUnit->Motor[3].EnableMotor();tx_semaphore_get(&CANSem,5);tx_thread_sleep(1);
	MotorUnit->Motor[4].EnableMotor();tx_semaphore_get(&CANSem,5);tx_thread_sleep(1);
	MotorUnit->Motor[5].EnableMotor();tx_semaphore_get(&CANSem,5);tx_thread_sleep(1);
	
////	MotorUnit->Motor[0].SetZero();tx_semaphore_get(&CANSem,5);tx_thread_sleep(100);
////	MotorUnit->Motor[3].SetZero();tx_semaphore_get(&CANSem,5);tx_thread_sleep(100);
//	MotorUnit->Motor[1].SetZero();tx_semaphore_get(&CANSem,5);tx_thread_sleep(100);
//	MotorUnit->Motor[2].SetZero();tx_semaphore_get(&CANSem,5);tx_thread_sleep(100);

	tx_thread_sleep(1000);
	MotorUnit->InitOdomentor();
	while(1)
	{	
		//500Hz		
		timess = tx_time_get();
		MotorUnit->Motor[0].MITTransmit();tx_semaphore_get(&CANSem,1);
		MotorUnit->Motor[1].MITTransmit();tx_semaphore_get(&CANSem,1);
		MotorUnit->Motor[2].MITTransmit();tx_semaphore_get(&CANSem,1);
		tx_thread_sleep(1);
		MotorUnit->Motor[3].MITTransmit();tx_semaphore_get(&CANSem,1);
		MotorUnit->Motor[4].MITTransmit();tx_semaphore_get(&CANSem,1);
		MotorUnit->Motor[5].MITTransmit();tx_semaphore_get(&CANSem,1);
		
		MotorUnit->UpdateLink();
		MotorUnit->UpdateOdomentor();
		
		tx_thread_sleep_until(&timess,2);	
	}
}

uint8_t RxData1[8];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,RxData1);
}

uint8_t RxData2[8];

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO1,&RxHeader,RxData2);
	
	if(RxHeader.Identifier==0x100)
	{
		MotorUnit->Motor[(uint8_t)(RxData2[0]&0x0F)-1].MessageDecode(RxData2);
		tx_semaphore_put(&CANSem);
	}
}


void cMotorUnit::UpdateLink(void)
{
	this->LinkSolver[0].InputLink( AngelCalib1+this->Motor[1].GetRadian(), AngelCalib0+this->Motor[0].GetRadian() );
	this->LinkSolver[1].InputLink( AngelCalib1-this->Motor[2].GetRadian(), AngelCalib0-this->Motor[3].GetRadian() );
	this->LinkSolver[0].Resolve();
	this->LinkSolver[1].Resolve();
}

void cMotorUnit::InitOdomentor(void)
{
	this->dlast[0] = this->Motor[4].GetRadian();
	this->dlast[1] = this->Motor[5].GetRadian();
	
	this->velocity = 0.0f;
	this->displacement = 0.0f;
	
	this->FilterV.CleanBuf();
	this->FilterD.CleanBuf();
}

void cMotorUnit::UpdateOdomentor(void)
{
	/*Calculate velocity firstly*/
	float tmp[2] = {0};
	tmp[0] = 0.5f * (this->Motor[4].GetVelocity() - this->Motor[5].GetVelocity());
	this->velocity = WHEELCOEFF*this->FilterV.BTW2Cal(tmp[0]);
	
	/*Then Calculate displacement*/
	tmp[0] = this->Motor[4].GetRadian() - this->dlast[0];
	tmp[1] = this->Motor[5].GetRadian() - this->dlast[1];

	this->dlast[0] = this->Motor[4].GetRadian();
	this->dlast[1] = this->Motor[5].GetRadian();
		
	if(tmp[0]>P_MAX){tmp[0]-=P_MAX;}
	else if(tmp[0]<P_MIN){tmp[0]+=P_MIN;}
	
	if(tmp[1]>P_MAX){tmp[1]-=P_MAX;}
	else if(tmp[1]<P_MIN){tmp[1]+=P_MIN;}
	
	this->displacement += (WHEELCOEFF*this->FilterD.BTW2Cal(0.5f*(tmp[0]-tmp[1])) );
}

void CANFilterConfig(void)
{
	FDCAN_FilterTypeDef Filter;
	Filter.IdType = FDCAN_STANDARD_ID;	
	Filter.FilterIndex = 0;
	Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	Filter.FilterType = FDCAN_FILTER_MASK;
	Filter.FilterID1 = 0x0000;
	Filter.FilterID2 = 0x0000;

	HAL_FDCAN_ConfigFilter(&hfdcan1,&Filter);
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	
	Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	HAL_FDCAN_ConfigFilter(&hfdcan2,&Filter);
	HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
}